using System.Collections.Generic;
using Meta.XR.MRUtilityKit;
using UnityEngine;

/// <summary>
/// Lets the player reach across the real table without the system boundary blanking the app, and replaces
/// the boundary's protection with the room scan:
///  - Asks Horizon OS to hide the boundary (Boundary API, "contextual boundaryless") while passthrough is
///    running. The OS only allows this with an active passthrough layer; this project keeps one under the
///    Moon world. Needs Boundary Visibility support in the Meta project config (manifest permission
///    com.oculus.permission.BOUNDARY_VISIBILITY).
///  - Marks every scanned piece of furniture in the virtual world, so the player avoids it without leaving
///    the Moon garden: by default each one gets a garden stand-in that fills the real object's box (a mossy
///    boulder, a bench for a couch or bed, a stone lantern for a lamp or plant), and the usu's table becomes a
///    wooden block whose top is exactly the real table top. <see cref="FurnitureStyle.Passthrough"/> cuts them
///    out to passthrough instead.
///  - Shows each real wall, door and window through passthrough when the head comes within
///    <see cref="wallRevealDistance"/> of it, so the player sees the wall before walking into it.
/// The boundary is only hidden after the room has loaded and the furniture is marked.
/// </summary>
public class MochiRoomSafety : MonoBehaviour
{
    public enum FurnitureStyle { StandIns, Passthrough }

    public bool suppressBoundary = true;
    public FurnitureStyle furnitureStyle = FurnitureStyle.StandIns;
    [Tooltip("Show a wall through passthrough when the head is closer than this to it, m.")]
    public float wallRevealDistance = 0.8f;
    [Tooltip("Material using Mochi/PassthroughCutout. Loaded from Resources/MochiHandCutout if empty.")]
    public Material cutoutMaterial;
    public Transform head;
    [Tooltip("Furniture whose box reaches into this space around the usu (radius, height above the usu base, m): in " +
             "Passthrough style it is not cut out (a cut-out would hide the usu); as a stand-in it is kept below the table top.")]
    public Vector2 usuClearance = new Vector2(0.45f, 0.7f);

    [Header("Stand-ins")]
    [Tooltip("Default stand-in (e.g. 06_Mossy_Stones_And_Leaves).")]
    public GameObject boulderProp;
    [Tooltip("Couches and beds (e.g. 04_Bench_With_Basket).")]
    public GameObject benchProp;
    [Tooltip("Lamps, plants and other tall, narrow things (e.g. 02_Stone_Lantern).")]
    public GameObject lanternProp;
    [Tooltip("Fallback for a missing prop, and the table block's sides (e.g. MR_PedestalBark).")]
    public Material standInSide;
    [Tooltip("The table block's top (e.g. MR_PedestalTop).")]
    public Material standInTop;
    [Tooltip("Props are scaled this much beyond the real footprint, so round props still cover a box's corners.")]
    public float footprintGrow = 1.15f;
    public bool tableStandIn = true;

    const MRUKAnchor.SceneLabels Structure = MRUKAnchor.SceneLabels.FLOOR | MRUKAnchor.SceneLabels.CEILING |
        MRUKAnchor.SceneLabels.WALL_FACE | MRUKAnchor.SceneLabels.INVISIBLE_WALL_FACE | MRUKAnchor.SceneLabels.INNER_WALL_FACE |
        MRUKAnchor.SceneLabels.DOOR_FRAME | MRUKAnchor.SceneLabels.WINDOW_FRAME | MRUKAnchor.SceneLabels.GLOBAL_MESH;
    const MRUKAnchor.SceneLabels Walls = MRUKAnchor.SceneLabels.WALL_FACE | MRUKAnchor.SceneLabels.INVISIBLE_WALL_FACE |
        MRUKAnchor.SceneLabels.INNER_WALL_FACE | MRUKAnchor.SceneLabels.DOOR_FRAME | MRUKAnchor.SceneLabels.WINDOW_FRAME;
    const MRUKAnchor.SceneLabels Seats = MRUKAnchor.SceneLabels.COUCH | MRUKAnchor.SceneLabels.BED;
    const MRUKAnchor.SceneLabels Tall = MRUKAnchor.SceneLabels.LAMP | MRUKAnchor.SceneLabels.PLANT;

    readonly List<(MRUKAnchor anchor, GameObject cut)> walls = new();
    readonly List<GameObject> furniture = new();
    bool roomReady;
    UsuTablePlacer placer;
    MRUKAnchor builtForTable;
    int skippedNearUsu, clippedNearUsu;

    System.Collections.IEnumerator Start()
    {
        placer = FindAnyObjectByType<UsuTablePlacer>();
        // Stand-in style: the table gets a wooden block here instead of the placer's passthrough cut-out.
        if (placer != null && furnitureStyle == FurnitureStyle.StandIns && tableStandIn) placer.cutOutTable = false;
        if (head == null && Camera.main != null) head = Camera.main.transform;
        if (cutoutMaterial == null) cutoutMaterial = Resources.Load<Material>("MochiHandCutout");
        OVRManager.BoundaryVisibilityChanged += v => Debug.Log($"MochiRoomSafety: system boundary is now {v}.");
        float t = 0f;
        while (MRUK.Instance == null && t < 2f) { t += Time.deltaTime; yield return null; }
        if (MRUK.Instance == null) { Debug.LogWarning("MochiRoomSafety: no MRUK; boundary stays on."); yield break; }
        MRUK.Instance.RegisterSceneLoadedCallback(Build);
    }

    void OnDisable() { if (OVRManager.instance != null) OVRManager.instance.shouldBoundaryVisibilityBeSuppressed = false; }

    void Build()
    {
        foreach (var go in furniture) if (go != null) Destroy(go);
        foreach (var w in walls) if (w.cut != null) Destroy(w.cut);
        furniture.Clear(); walls.Clear();
        skippedNearUsu = clippedNearUsu = 0;
        builtForTable = placer != null ? placer.Table : null;
        if (cutoutMaterial == null) { Debug.LogWarning("MochiRoomSafety: no cutout material; boundary stays on."); return; }
        bool standIns = furnitureStyle == FurnitureStyle.StandIns;

        foreach (var room in MRUK.Instance.Rooms)
            foreach (var a in room.Anchors)
            {
                if (a == null) continue;
                if (a.HasAnyLabel(Walls) && a.PlaneRect is Rect r)
                {
                    // Thin box rather than a quad so it draws from either side.
                    var cut = Cutout(a.transform, new Vector3(r.center.x, r.center.y, 0f), new Vector3(r.width, r.height, 0.01f), "WallCutout");
                    cut.SetActive(false);
                    walls.Add((a, cut));
                    continue;
                }
                if (a.HasAnyLabel(Structure) || !(a.VolumeBounds is Bounds b)) continue;

                bool isTable = placer != null && placer.Table == a;
                if (isTable)
                {
                    if (standIns && tableStandIn)
                    {
                        placer.ClearTableCutout();
                        furniture.Add(TableBlock(a, b));
                    }
                    continue;   // otherwise UsuTablePlacer's cut-out shows the real table
                }

                bool near = NearUsu(a.transform, b);
                if (!standIns)
                {
                    if (near) { skippedNearUsu++; continue; }
                    furniture.Add(Cutout(a.transform, b.center, b.size, "FurnitureCutout"));
                    continue;
                }
                var s = StandIn(a, b, near);
                if (s != null) furniture.Add(s);
            }
        roomReady = true;
        Debug.Log(standIns
            ? $"MochiRoomSafety: {furniture.Count} garden stand-ins ({clippedNearUsu} kept below the usu table), {walls.Count} walls revealed within {wallRevealDistance} m."
            : $"MochiRoomSafety: {furniture.Count} furniture cut-outs ({skippedNearUsu} skipped next to the usu), {walls.Count} walls revealed within {wallRevealDistance} m.");
    }

    // ------------------------------------------------------------------ stand-ins

    /// <summary>The anchor's volume as an upright box: centre of the base, yaw, size (x, height, z).</summary>
    static void UprightBox(Transform a, Bounds local, out Vector3 baseCentre, out Quaternion yaw, out Vector3 size)
    {
        // Yaw along whichever local axis is most horizontal, so the box hugs the real object.
        Vector3 fwd = Vector3.zero; float best = -1f;
        foreach (var axis in new[] { a.right, a.up, a.forward })
        {
            var h = Vector3.ProjectOnPlane(axis, Vector3.up);
            if (h.sqrMagnitude > best) { best = h.sqrMagnitude; fwd = h; }
        }
        yaw = Quaternion.LookRotation(fwd.normalized, Vector3.up);
        var inv = Quaternion.Inverse(yaw);
        Vector3 mn = Vector3.positiveInfinity, mx = Vector3.negativeInfinity;
        Vector3 c = local.center, e = local.extents;
        for (int i = 0; i < 8; i++)
        {
            var corner = c + Vector3.Scale(e, new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1));
            var p = inv * (a.TransformPoint(corner) - a.position);
            mn = Vector3.Min(mn, p); mx = Vector3.Max(mx, p);
        }
        size = mx - mn;
        var mid = (mn + mx) * 0.5f; mid.y = mn.y;
        baseCentre = a.position + yaw * mid;
    }

    GameObject StandIn(MRUKAnchor a, Bounds b, bool nearUsu)
    {
        UprightBox(a.transform, b, out var basePos, out var yaw, out var size);
        if (nearUsu && placer != null)
        {
            // Never rise into the usu's working space: a chair tucked under the table stays under its top.
            float maxH = placer.transform.position.y - 0.02f - basePos.y;
            if (maxH < 0.05f) return null;
            if (size.y > maxH) { size.y = maxH; clippedNearUsu++; }
        }

        float foot = Mathf.Max(size.x, size.z);
        GameObject prop = a.HasAnyLabel(Seats) ? benchProp
                        : a.HasAnyLabel(Tall) || (size.y > 1.3f && foot < 0.6f) ? lanternProp
                        : boulderProp;
        if (prop == null) prop = boulderProp;

        var holder = new GameObject($"StandIn_{a.name}");
        holder.transform.SetParent(a.transform, true);
        holder.transform.SetPositionAndRotation(basePos, yaw);
        Vector3 target = new Vector3(size.x * footprintGrow, size.y, size.z * footprintGrow);
        if (prop != null) FitProp(prop, holder.transform, target);
        else Block(holder.transform, target, standInSide, standInSide);
        return holder;
    }

    /// <summary>Instantiates the prop under the holder, stretched so its bounds fill the box (base at the holder).</summary>
    static void FitProp(GameObject prop, Transform holder, Vector3 target)
    {
        var go = Instantiate(prop);
        go.name = prop.name;
        foreach (var c in go.GetComponentsInChildren<Collider>()) Destroy(c);
        go.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
        go.transform.localScale = Vector3.one;
        var rs = go.GetComponentsInChildren<Renderer>();
        if (rs.Length == 0) { Destroy(go); return; }
        var bb = rs[0].bounds;
        foreach (var r in rs) { bb.Encapsulate(r.bounds); r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off; }
        var k = new Vector3(target.x / Mathf.Max(1e-3f, bb.size.x), target.y / Mathf.Max(1e-3f, bb.size.y), target.z / Mathf.Max(1e-3f, bb.size.z));
        var root = new GameObject("Fit").transform;           // scale about the prop's base centre
        root.SetParent(holder, false);
        go.transform.SetParent(root, false);
        go.transform.localPosition = -new Vector3(bb.center.x, bb.min.y, bb.center.z);
        root.localScale = k;
    }

    /// <summary>A box with its base at the holder: sides, plus a separate top face.</summary>
    static void Block(Transform holder, Vector3 size, Material side, Material top)
    {
        var body = GameObject.CreatePrimitive(PrimitiveType.Cube);
        body.name = "Block";
        Destroy(body.GetComponent<Collider>());
        body.transform.SetParent(holder, false);
        body.transform.localPosition = new Vector3(0f, size.y * 0.5f, 0f);
        body.transform.localScale = size;
        if (side != null) body.GetComponent<MeshRenderer>().sharedMaterial = side;
        if (top == null || top == side) return;
        var cap = GameObject.CreatePrimitive(PrimitiveType.Quad);
        cap.name = "Top";
        Destroy(cap.GetComponent<Collider>());
        cap.transform.SetParent(holder, false);
        cap.transform.localPosition = new Vector3(0f, size.y + 0.001f, 0f);
        cap.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
        cap.transform.localScale = new Vector3(size.x, size.z, 1f);
        cap.GetComponent<MeshRenderer>().sharedMaterial = top;
    }

    /// <summary>The usu's table as a wooden block exactly the size of the real one, so the usu sits on its top.</summary>
    GameObject TableBlock(MRUKAnchor a, Bounds b)
    {
        UprightBox(a.transform, b, out var basePos, out var yaw, out var size);
        // Down to the floor even if the scan's box stops short.
        float floor = a.Room != null && a.Room.FloorAnchors != null && a.Room.FloorAnchors.Count > 0 && a.Room.FloorAnchors[0] != null
            ? a.Room.FloorAnchors[0].transform.position.y : basePos.y;
        float top = basePos.y + size.y;
        if (floor < basePos.y) { basePos.y = floor; size.y = top - floor; }
        var holder = new GameObject("StandIn_UsuTable");
        holder.transform.SetParent(a.transform, true);
        holder.transform.SetPositionAndRotation(basePos, yaw);
        Block(holder.transform, size, standInSide, standInTop != null ? standInTop : standInSide);
        return holder;
    }

    // ------------------------------------------------------------------ cut-outs

    // True if the anchor's box comes within the usu's working space (a cylinder above the usu base).
    bool NearUsu(Transform anchor, Bounds local)
    {
        if (placer == null || placer.Table == null) return false;
        Transform usu = placer.transform;
        Vector3 c = local.center, e = local.extents;
        Vector3 mn = Vector3.positiveInfinity, mx = Vector3.negativeInfinity;
        for (int i = 0; i < 8; i++)
        {
            var corner = c + Vector3.Scale(e, new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1));
            var p = usu.InverseTransformPoint(anchor.TransformPoint(corner));
            mn = Vector3.Min(mn, p); mx = Vector3.Max(mx, p);
        }
        if (mx.y < 0.005f || mn.y > usuClearance.y) return false;   // entirely below the table top or far above
        float dx = Mathf.Max(mn.x, Mathf.Min(0f, mx.x)), dz = Mathf.Max(mn.z, Mathf.Min(0f, mx.z));
        return dx * dx + dz * dz < usuClearance.x * usuClearance.x;
    }

    GameObject Cutout(Transform parent, Vector3 localCentre, Vector3 size, string name)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        go.name = name;
        Destroy(go.GetComponent<Collider>());
        var mr = go.GetComponent<MeshRenderer>();
        mr.sharedMaterial = cutoutMaterial;
        mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        mr.receiveShadows = false;
        go.transform.SetParent(parent, false);
        go.transform.localPosition = localCentre;
        go.transform.localRotation = Quaternion.identity;
        go.transform.localScale = size;
        return go;
    }

    void Update()
    {
        // The usu may be placed after the room loads: rebuild so the furniture respects where it ended up.
        if (roomReady && placer != null && placer.Table != builtForTable) Build();
        if (head != null)
            foreach (var (anchor, cut) in walls)
            {
                if (anchor == null || cut == null) continue;
                bool near = anchor.GetClosestSurfacePosition(head.position, out _) < wallRevealDistance;
                if (cut.activeSelf != near) cut.SetActive(near);
            }

        var m = OVRManager.instance;
        if (m == null) return;
        var pt = FindPassthrough();
        bool want = suppressBoundary && roomReady && m.isInsightPassthroughEnabled && pt != null && pt.isActiveAndEnabled;
        if (m.shouldBoundaryVisibilityBeSuppressed != want) m.shouldBoundaryVisibilityBeSuppressed = want;
    }

    OVRPassthroughLayer passthrough;
    OVRPassthroughLayer FindPassthrough() => passthrough != null ? passthrough : (passthrough = FindAnyObjectByType<OVRPassthroughLayer>());
}
