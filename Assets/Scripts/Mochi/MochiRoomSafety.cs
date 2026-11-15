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
///  - Cuts every scanned piece of furniture out of the virtual world, so the real objects show in
///    passthrough where they stand (the usu's table is cut out by UsuTablePlacer).
///  - Shows each real wall, door and window through passthrough when the head comes within
///    <see cref="wallRevealDistance"/> of it, so the player sees the wall before walking into it.
/// The boundary is only hidden after the room has loaded and those cut-outs exist.
/// </summary>
public class MochiRoomSafety : MonoBehaviour
{
    public bool suppressBoundary = true;
    public bool cutOutFurniture = true;
    [Tooltip("Show a wall through passthrough when the head is closer than this to it, m.")]
    public float wallRevealDistance = 0.8f;
    [Tooltip("Material using Mochi/PassthroughCutout. Loaded from Resources/MochiHandCutout if empty.")]
    public Material cutoutMaterial;
    public Transform head;
    [Tooltip("Furniture whose box reaches into this space around the usu (radius, height above the usu base, m) is not cut out, " +
             "so cut-outs never hide the usu, dough, kine or ghost hands. The player is at the table there anyway.")]
    public Vector2 usuClearance = new Vector2(0.45f, 0.7f);

    const MRUKAnchor.SceneLabels Structure = MRUKAnchor.SceneLabels.FLOOR | MRUKAnchor.SceneLabels.CEILING |
        MRUKAnchor.SceneLabels.WALL_FACE | MRUKAnchor.SceneLabels.INVISIBLE_WALL_FACE | MRUKAnchor.SceneLabels.INNER_WALL_FACE |
        MRUKAnchor.SceneLabels.DOOR_FRAME | MRUKAnchor.SceneLabels.WINDOW_FRAME | MRUKAnchor.SceneLabels.GLOBAL_MESH;
    const MRUKAnchor.SceneLabels Walls = MRUKAnchor.SceneLabels.WALL_FACE | MRUKAnchor.SceneLabels.INVISIBLE_WALL_FACE |
        MRUKAnchor.SceneLabels.INNER_WALL_FACE | MRUKAnchor.SceneLabels.DOOR_FRAME | MRUKAnchor.SceneLabels.WINDOW_FRAME;

    readonly List<(MRUKAnchor anchor, GameObject cut)> walls = new();
    readonly List<GameObject> furniture = new();
    bool roomReady;
    UsuTablePlacer placer;
    MRUKAnchor builtForTable;
    int skippedNearUsu;

    System.Collections.IEnumerator Start()
    {
        placer = FindAnyObjectByType<UsuTablePlacer>();
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
        skippedNearUsu = 0;
        builtForTable = placer != null ? placer.Table : null;
        if (cutoutMaterial == null) { Debug.LogWarning("MochiRoomSafety: no cutout material; boundary stays on."); return; }

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
                }
                else if (cutOutFurniture && !a.HasAnyLabel(Structure) && a.VolumeBounds is Bounds b
                         && (placer == null || placer.Table != a))
                {
                    if (NearUsu(a.transform, b)) { skippedNearUsu++; continue; }
                    furniture.Add(Cutout(a.transform, b.center, b.size, "FurnitureCutout"));
                }
            }
        roomReady = true;
        Debug.Log($"MochiRoomSafety: {furniture.Count} furniture cut-outs ({skippedNearUsu} skipped next to the usu), {walls.Count} walls revealed within {wallRevealDistance} m.");
    }

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
        // The usu may be placed after the room loads: rebuild so cut-outs respect where it ended up.
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
