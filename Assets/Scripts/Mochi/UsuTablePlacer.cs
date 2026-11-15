using System.Collections;
using System.Collections.Generic;
using Meta.XR.MRUtilityKit;
using UnityEngine;

/// <summary>
/// Puts the MochiStation (the usu's bottom centre) on the real table nearest the player, using the room
/// scan from MR Utility Kit (Space Setup). The usu sits on the table top at the point of the table closest
/// to the player, pulled in from the edges so the whole usu is on the table, and faces away from the player.
///
/// If the room has no scan, or the scan has no table, the station is left where it is (or put in front of
/// the head if <see cref="MochiStation.placeInFrontOfHead"/> is set), and the player can be sent to Space
/// Setup once to add a table.
///
/// Needs an MRUK component in the scene (with World Lock off: this project moves the camera rig itself).
/// </summary>
[RequireComponent(typeof(MochiStation))]
public class UsuTablePlacer : MonoBehaviour
{
    [Tooltip("Usu footprint radius on the table, m (outer radius of the usu body plus a small margin).")]
    public float footprintRadius = 0.25f;
    [Tooltip("How far in from the table's edge nearest the player the usu's centre goes, beyond the footprint, m.")]
    public float edgeInset = 0.02f;
    [Tooltip("Ignore tables whose top is farther than this from the head, horizontally, m.")]
    public float maxTableDistance = 3f;
    [Tooltip("Ignore surfaces lower than this above the floor (benches, footstools), m.")]
    public float minTableHeight = 0.45f;
    [Tooltip("When the scan has no table, ask the system to open Space Setup once so the player can add one.")]
    public bool requestSpaceSetupIfNoTable = true;
    [Tooltip("Seconds to wait for a head pose before measuring which table is nearest.")]
    public float headWait = 0.5f;
    [Tooltip("Cut the table's volume out of the virtual world so the real table shows in passthrough under the usu " +
             "(otherwise, in a VR environment, the usu floats and the real table is invisible to walk into).")]
    public bool cutOutTable = true;
    [Tooltip("Material using Mochi/PassthroughCutout. Loaded from Resources/MochiHandCutout if empty.")]
    public Material cutoutMaterial;
    [Tooltip("Virtual stands for the usu (e.g. the Moon environment's MR_UsuPedestal) switched off once the usu is on a real table: " +
             "otherwise they stand in the room where nothing real is and get in the player's way. Found by name if empty.")]
    public GameObject[] hideWhenOnTable;
    public string[] hideNames = { "MR_UsuPedestal" };

    MochiStation station;
    bool spaceSetupRequested;
    GameObject tableCutout;

    /// <summary>The table the usu was last put on, or null.</summary>
    public MRUKAnchor Table { get; private set; }

    void Awake() => station = GetComponent<MochiStation>();

    IEnumerator Start()
    {
        float t = 0f;
        while (MRUK.Instance == null && t < 2f) { t += Time.deltaTime; yield return null; }
        if (MRUK.Instance == null) { Debug.LogWarning("UsuTablePlacer: no MRUK in the scene; usu stays where it is."); yield break; }
        // Let tracking deliver a real head pose first, so "nearest" means nearest to the player.
        yield return new WaitForSeconds(headWait);
        MRUK.Instance.RegisterSceneLoadedCallback(OnSceneLoaded);
        // A table added later (e.g. after Space Setup) is used, but a placed usu never jumps mid-session.
        MRUK.Instance.RoomUpdatedEvent.AddListener(_ => { if (Table == null) OnSceneLoaded(); });
    }

    void OnDestroy()
    {
        if (MRUK.Instance == null) return;
        MRUK.Instance.SceneLoadedEvent.RemoveListener(OnSceneLoaded);
    }

    void OnSceneLoaded()
    {
        if (PlaceOnNearestTable()) return;
        if (requestSpaceSetupIfNoTable && !spaceSetupRequested && !Application.isEditor)
        {
            spaceSetupRequested = true;
            RequestSpaceSetup();
        }
    }

    async void RequestSpaceSetup()
    {
        Debug.Log("UsuTablePlacer: no table in the room scan, opening Space Setup.");
        bool ok = await OVRScene.RequestSpaceSetup();
        if (!ok || MRUK.Instance == null) return;
        await MRUK.Instance.LoadSceneFromDevice(requestSceneCaptureIfNoDataFound: false);
        OnSceneLoaded();
    }

    /// <summary>Moves the station onto the nearest scanned table. Returns false if there is none.</summary>
    [ContextMenu("Place On Nearest Table")]
    public bool PlaceOnNearestTable()
    {
        if (MRUK.Instance == null) return false;
        var head = station.head != null ? station.head : (Camera.main != null ? Camera.main.transform : null);
        if (head == null) return false;
        Vector3 eye = head.position;

        MRUKAnchor best = null; Vector3 bestPoint = default; float bestDist = float.MaxValue;
        foreach (var room in MRUK.Instance.Rooms)
        {
            float floorY = FloorY(room, float.NegativeInfinity);
            foreach (var anchor in room.Anchors)
            {
                if (anchor == null || !anchor.HasAnyLabel(MRUKAnchor.SceneLabels.TABLE)) continue;
                if (!TryTablePoint(anchor, eye, out Vector3 p)) continue;
                if (p.y - floorY < minTableHeight) continue;
                float d = Vector2.Distance(new Vector2(p.x, p.z), new Vector2(eye.x, eye.z));
                if (d > maxTableDistance || d >= bestDist) continue;
                best = anchor; bestPoint = p; bestDist = d;
            }
        }
        if (best == null)
        {
            Debug.Log($"UsuTablePlacer: no table within {maxTableDistance} m in {MRUK.Instance.Rooms.Count} room(s).");
            return false;
        }

        // Face away from the player, so the kneader stands at the near side as before.
        Vector3 fwd = bestPoint - eye; fwd.y = 0f;
        if (fwd.sqrMagnitude < 1e-4f) fwd = Vector3.ProjectOnPlane(head.forward, Vector3.up);
        if (fwd.sqrMagnitude < 1e-4f) fwd = Vector3.forward;
        station.transform.SetPositionAndRotation(bestPoint, Quaternion.LookRotation(fwd.normalized, Vector3.up));
        Table = best;
        if (cutOutTable) BuildCutout(best);
        HideStands();
        Debug.Log($"UsuTablePlacer: usu on '{best.name}' at {bestPoint} ({bestDist:F2} m from the head, table top {bestPoint.y:F2} m).");
        return true;
    }

    /// <summary>Height of the room's lowest floor anchor, or fallback if it has none.</summary>
    static float FloorY(MRUKRoom room, float fallback)
    {
        if (room == null || room.FloorAnchors == null || room.FloorAnchors.Count == 0) return fallback;
        float y = float.PositiveInfinity;
        foreach (var f in room.FloorAnchors) if (f != null) y = Mathf.Min(y, f.transform.position.y);
        return float.IsPositiveInfinity(y) ? fallback : y;
    }

    void HideStands()
    {
        var list = new List<GameObject>();
        if (hideWhenOnTable != null) list.AddRange(hideWhenOnTable);
        if (list.Count == 0 && hideNames != null)
            foreach (var n in hideNames) { var go = GameObject.Find(n); if (go != null) list.Add(go); }
        foreach (var go in list)
            if (go != null && go.activeSelf) { go.SetActive(false); Debug.Log($"UsuTablePlacer: hid '{go.name}' (usu is on a real table)."); }
    }

    /// <summary>Removes the table's passthrough cut-out (another component shows the table instead).</summary>
    public void ClearTableCutout()
    {
        if (tableCutout != null) Destroy(tableCutout);
        tableCutout = null;
    }

    /// <summary>A box over the table's volume (or its top, down to the floor) drawn with the passthrough cutout.</summary>
    void BuildCutout(MRUKAnchor table)
    {
        if (tableCutout != null) Destroy(tableCutout);
        if (cutoutMaterial == null) cutoutMaterial = Resources.Load<Material>("MochiHandCutout");
        if (cutoutMaterial == null) { Debug.LogWarning("UsuTablePlacer: no cutout material, table not shown."); return; }

        Bounds b;
        if (table.VolumeBounds is Bounds vb) b = vb;
        else if (table.PlaneRect is Rect pr)
        {
            // Plane only: extend from the top down to the floor along the local axis that points down.
            float depth = table.transform.position.y - FloorY(table.Room, table.transform.position.y - 0.75f);
            Vector3 down = table.transform.InverseTransformDirection(Vector3.down);
            b = new Bounds(new Vector3(pr.center.x, pr.center.y, 0f), new Vector3(pr.width, pr.height, 0f));
            b.Encapsulate(b.center + down * depth);
        }
        else return;

        tableCutout = GameObject.CreatePrimitive(PrimitiveType.Cube);
        tableCutout.name = "TableCutout";
        Destroy(tableCutout.GetComponent<Collider>());
        var r = tableCutout.GetComponent<MeshRenderer>();
        r.sharedMaterial = cutoutMaterial;
        r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        r.receiveShadows = false;
        var t = tableCutout.transform;
        t.SetParent(table.transform, false);
        t.localPosition = b.center;
        t.localRotation = Quaternion.identity;
        t.localScale = b.size;
    }

    /// <summary>
    /// The point on the table's top surface nearest the eye (horizontally), kept far enough from the edges
    /// for the usu to fit. Works for plane and volume table anchors, whichever local axis is up.
    /// </summary>
    bool TryTablePoint(MRUKAnchor anchor, Vector3 eye, out Vector3 point)
    {
        point = default;
        var tr = anchor.transform;
        Vector3 min, max;
        if (anchor.VolumeBounds is Bounds vb) { min = vb.min; max = vb.max; }
        else if (anchor.PlaneRect is Rect pr) { min = new Vector3(pr.xMin, pr.yMin, 0f); max = new Vector3(pr.xMax, pr.yMax, 0f); }
        else return false;

        // Local axis that points most nearly up in the world: the table top is at its far end.
        int up = 0; float upDot = 0f;
        for (int a = 0; a < 3; a++)
        {
            Vector3 axis = Vector3.zero; axis[a] = 1f;
            float d = Vector3.Dot(tr.TransformDirection(axis), Vector3.up);
            if (Mathf.Abs(d) > Mathf.Abs(upDot)) { up = a; upDot = d; }
        }
        if (Mathf.Abs(upDot) < 0.9f) return false; // tilted: not a usable table top

        Vector3 local = tr.InverseTransformPoint(eye);
        float inset = footprintRadius + edgeInset;
        for (int a = 0; a < 3; a++)
        {
            if (a == up) { local[a] = upDot > 0f ? max[a] : min[a]; continue; }
            float lo = min[a] + inset, hi = max[a] - inset;
            local[a] = lo <= hi ? Mathf.Clamp(local[a], lo, hi) : 0.5f * (min[a] + max[a]);
        }
        point = tr.TransformPoint(local);
        return true;
    }
}
