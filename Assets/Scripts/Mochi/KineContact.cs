using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Collisions between the kine and the other solid things, which the MPM solver does not do (it only
/// collides the dough with tools; tools pass through each other). After whatever moved the kine this
/// frame (MochiGrabber, KineDriver, the rhythm partner):
///  - the head is pushed out of the usu (the cross-section UsuTool's visual is lathed from: outer wall,
///    rim, inner bowl, floor), the table top around it, and free tracked hands while the kine moves;
///  - the handle is kept out of the usu rim by pivoting the kine, about the holding hand or controller
///    when held (so it stays in the hand and the head swings up), about the head when the kine drives
///    itself (KineDriver's auto-pound already tips the handle over the rim, so this only mops up);
///  - the handle is also kept off the table (and, if enabled, free hands) by pushing it out.
///
/// Real hands cannot be pushed back, so a held kine stops at the surface while the hand carries on, and
/// snaps back to the hand when it moves away. Position and rotation are swept in small steps so a fast
/// swing cannot pass through the rim. A resting kine is not shoved around by hands. Contacts raise
/// <see cref="Contact"/> and buzz the holding controller. Idle while MochiStrikeRecorder or MochiTestBench runs.
///
/// Added at runtime to every KineTool (see <see cref="KineContactBootstrap"/>) unless one is already in the
/// scene; define MOCHI_KINE_CONTACT_OFF to stop that.
/// </summary>
[DefaultExecutionOrder(-55)] // after MochiGrabber (-100), RhythmPartner (-70), KineDriver (-60); before KineTool (-50) gives the pose to the solver
[RequireComponent(typeof(KineTool))]
public class KineContact : MonoBehaviour
{
    public enum Kind { Usu, Table, Hand }

    public MochiStation station;
    public UsuTool usu;

    [Header("What the kine head hits")]
    public bool hitUsu = true;
    public bool hitTable = true;
    public bool hitHands = true;

    [Header("What the handle hits")]
    [Tooltip("The handle rests on the rim instead of passing through it; the kine pivots about the hand (held) or the head (driven).")]
    public bool handleHitsUsu = true;
    public bool handleHitsTable = true;
    [Tooltip("Off by default: the handle reaches over the kneader's wrists.")]
    public bool handleHitsHands = false;
    [Tooltip("Handle radius, m (HandleVisual is 3.5 cm across).")]
    public float handleRadius = 0.0175f;

    [Header("Table (station space; the usu stands on it at y = 0, +Z away from the kneader)")]
    public float tableHalfWidth = 0.6f;
    public float tableFar = 0.6f;
    [Tooltip("Station z of the table's near edge. NaN: from UsuTablePlacer (footprintRadius + edgeInset in front of the usu), else 0.3 m in front.")]
    public float tableNear = float.NaN;

    [Header("Feel")]
    [Tooltip("Longest step (m) a frame's motion of the head or the handle end is swept in, so fast swings cannot pass through the rim.")]
    public float maxStep = 0.02f;
    [Tooltip("Approach speed (m/s) above which a contact counts as an impact (event + haptics).")]
    public float impactSpeed = 0.5f;
    [Range(0f, 1f)] public float hapticAmplitude = 0.6f;
    public float hapticSeconds = 0.06f;

    /// <summary>Impact: world point, approach speed (m/s), what was hit.</summary>
    public event Action<Vector3, float, Kind> Contact;
    public bool InContact { get; private set; }

    KineTool kine;
    GrabZone zone;
    MochiGrabber grabber;
    readonly List<Vector4> headBalls = new List<Vector4>();   // kine-local centre, radius
    readonly List<Vector4> handleBalls = new List<Vector4>();
    readonly List<Vector4> handBalls = new List<Vector4>();   // station-local centre, radius
    readonly Dictionary<MatTool, OVRHand> handFor = new Dictionary<MatTool, OVRHand>();
    Vector3 handleA, handleB; // handle axis ends, kine local
    Vector2[] profile;        // usu cross-section (radius, height above the usu base), counter-clockwise
    Vector3 usuBase;          // station-local position of the usu's base centre
    Vector3 prevPos;
    Quaternion prevRot = Quaternion.identity;
    bool hasPrev, suspended, stepTouched;
    float nextSuspendCheck, buzzUntil;
    OVRInput.Controller buzzing = OVRInput.Controller.None;
    Kind lastKind;
    Vector3 lastNormal, lastPoint;

    void Awake()
    {
        kine = GetComponent<KineTool>();
        zone = GetComponent<GrabZone>();
        if (station == null) station = GetComponentInParent<MochiStation>();
        if (station == null) station = FindAnyObjectByType<MochiStation>();
        if (usu == null && station != null) usu = station.sharedTools.OfType<UsuTool>().FirstOrDefault();
        if (usu == null && station != null) usu = station.GetComponentInChildren<UsuTool>(true);
        grabber = FindAnyObjectByType<MochiGrabber>();
        BuildShapes();
    }

    void Start()
    {
        BuildProfile();
        if (float.IsNaN(tableNear))
        {
            var placer = station != null ? station.GetComponent<UsuTablePlacer>() : null;
            tableNear = usuBase.z - (placer != null ? placer.footprintRadius + placer.edgeInset : 0.3f);
        }
    }

    // The head as a capsule core plus rings of small balls on the face and top edges (its cylinder corners);
    // the handle as a row of balls along the GrabZone segment.
    void BuildShapes()
    {
        float R = kine.headRadius, L = kine.headLength, f = Mathf.Clamp(kine.faceRounding, 0.008f, R * 0.5f);
        int core = Mathf.Max(2, Mathf.CeilToInt((L - 2f * R) / R) + 1);
        for (int i = 0; i < core; i++)
            headBalls.Add(new Vector4(0f, Mathf.Lerp(R, Mathf.Max(R, L - R), i / (float)(core - 1)), 0f, R));
        for (int ring = 0; ring < 2; ring++)
            for (int i = 0; i < 8; i++)
            {
                float a = i * Mathf.PI / 4f;
                headBalls.Add(new Vector4(Mathf.Cos(a) * (R - f), ring == 0 ? f : L - f, Mathf.Sin(a) * (R - f), f));
            }
        Vector3 a0 = zone != null ? zone.segmentA : new Vector3(0f, 0.1f, 0f);
        Vector3 b0 = zone != null ? zone.segmentB : new Vector3(0f, 0.1f, -0.63f);
        handleA = a0;
        handleB = b0;
        Vector3 dir = (b0 - a0).normalized;
        Vector3 start = a0 + dir * R; // the part inside the head is covered by the head
        float len = Vector3.Distance(start, b0);
        int n = Mathf.Max(2, Mathf.CeilToInt(len / handleRadius) + 1);
        for (int i = 0; i < n; i++)
        {
            Vector3 p = Vector3.Lerp(start, b0, i / (float)(n - 1));
            handleBalls.Add(new Vector4(p.x, p.y, p.z, handleRadius));
        }
    }

    // UsuTool.BuildLatheMesh's profile, closed downward into the table so the base and the table are one solid.
    void BuildProfile()
    {
        float ro = 0.205f, rTop = 0.15f, rBot = 0.125f, floorY = 0.03f, depth = 0.13f;
        usuBase = Vector3.zero;
        if (usu != null)
        {
            ro = usu.outerRadius; rTop = usu.innerRadiusTop; rBot = usu.innerRadiusBottom; floorY = usu.baseThickness; depth = usu.depth;
            if (station != null) usuBase = station.transform.InverseTransformPoint(usu.transform.position);
        }
        float rimY = floorY + depth + 0.02f;
        profile = new[]
        {
            new Vector2(0f, -1f), new Vector2(ro * 0.96f, -1f), new Vector2(ro * 0.96f, 0f), new Vector2(ro, 0.015f),
            new Vector2(ro, rimY - 0.01f), new Vector2(ro - 0.01f, rimY), new Vector2(rTop + 0.01f, rimY), new Vector2(rTop, rimY - 0.01f),
            new Vector2(Mathf.Lerp(rBot, rTop, 0.5f), floorY + depth * 0.5f), new Vector2(rBot, floorY + 0.015f),
            new Vector2(rBot - 0.02f, floorY), new Vector2(0f, floorY),
        };
    }

    // Only an enabled recorder or test bench counts: scenes keep a disabled MochiTestBench on the station,
    // and FindAnyObjectByType also returns disabled components.
    static bool Running<T>() where T : Behaviour
    {
        foreach (var o in FindObjectsByType<T>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
            if (o.isActiveAndEnabled) return true;
        return false;
    }

    void Update()
    {
        if (station == null || kine == null || profile == null) return;
        if (Time.unscaledTime >= nextSuspendCheck)
        {
            nextSuspendCheck = Time.unscaledTime + 1f;
            suspended = Running<MochiStrikeRecorder>() || Running<MochiTestBench>();
        }
        UpdateHaptics();
        if (suspended) { hasPrev = false; InContact = false; return; }

        Transform st = station.transform;
        float dt = Mathf.Max(Time.deltaTime, 1e-4f);
        Vector3 targetPos = st.InverseTransformPoint(transform.position);
        Quaternion targetRot = Quaternion.Inverse(st.rotation) * transform.rotation;
        bool held = zone != null && zone.Held;
        float moved = hasPrev ? (targetPos - prevPos).magnitude : 0f;
        float endMoved = hasPrev ? ((targetPos + targetRot * handleB) - (prevPos + prevRot * handleB)).magnitude : 0f;
        bool resting = !held && moved < 0.001f && endMoved < 0.001f; // a resting kine is not shoved around by hands
        bool hands = (hitHands || handleHitsHands) && !resting && GatherHands(st, held);
        Vector3 pivotWorld = default;
        bool hasPivot = held && HolderPoint(out pivotWorld);
        Vector3 pivotLocal = hasPivot ? st.InverseTransformPoint(pivotWorld) : default;

        stepTouched = false;
        Vector3 pos;
        Quaternion rot;
        // put down or teleported (KineDriver.Rest moves it ~0.45 m in one frame): no sweep. A 10 m/s swing is ~0.14 m a frame.
        if (!hasPrev || moved > 0.25f)
        {
            pos = targetPos;
            rot = targetRot;
            Step(ref pos, ref rot, hands, hasPivot, pivotLocal);
        }
        else
        {
            int n = Mathf.Clamp(Mathf.CeilToInt(Mathf.Max(moved, endMoved) / Mathf.Max(maxStep, 0.005f)), 1, 16);
            pos = prevPos;
            rot = prevRot;
            for (int k = 0; k < n; k++)
            {
                float t = 1f / (n - k);
                pos += (targetPos - pos) * t;
                rot = Quaternion.Slerp(rot, targetRot, t);
                Step(ref pos, ref rot, hands, hasPivot, pivotLocal);
            }
        }

        bool touching = stepTouched;
        if (touching) transform.SetPositionAndRotation(st.TransformPoint(pos), st.rotation * rot);
        if (touching && !InContact && hasPrev)
        {
            // how fast the contact point was coming at the surface (the same body point at last frame's and this frame's target pose)
            Vector3 body = Quaternion.Inverse(rot) * (lastPoint - pos);
            Vector3 before = prevPos + prevRot * body, after = targetPos + targetRot * body;
            float approach = Vector3.Dot(after - before, -lastNormal) / dt;
            if (approach > impactSpeed)
            {
                Contact?.Invoke(st.TransformPoint(lastPoint), approach, lastKind);
                Buzz(approach);
            }
        }
        InContact = touching;
        prevPos = pos;
        prevRot = rot;
        hasPrev = true;
    }

    /// <summary>One sweep step: push out by translation, then pivot the handle off the rim; twice, as each can disturb the other.</summary>
    void Step(ref Vector3 pos, ref Quaternion rot, bool hands, bool hasPivot, Vector3 pivotLocal)
    {
        for (int iter = 0; iter < 2; iter++)
        {
            pos = Resolve(pos, rot, hands);
            if (!handleHitsUsu || !PivotHandle(ref pos, ref rot, hasPivot, pivotLocal)) break;
        }
    }

    /// <summary>Pushes the kine (station-local origin, rotation) out of everything except handle-versus-usu, a few passes.</summary>
    Vector3 Resolve(Vector3 pos, Quaternion rot, bool hands)
    {
        for (int pass = 0; pass < 4; pass++)
        {
            float deepest = 1e-4f;
            Vector3 push = Vector3.zero, point = Vector3.zero;
            Kind kind = lastKind;
            Test(headBalls, pos, rot, hitUsu, hitTable, hands && hitHands, ref deepest, ref push, ref point, ref kind);
            Test(handleBalls, pos, rot, false, handleHitsTable, hands && handleHitsHands, ref deepest, ref push, ref point, ref kind);
            if (push.sqrMagnitude < 0.5f) break; // push is a unit normal, or zero when nothing is hit
            pos += push * (deepest + 1e-4f);
            Touch(kind, push, point);
        }
        return pos;
    }

    void Test(List<Vector4> balls, Vector3 pos, Quaternion rot, bool useUsu, bool useTable, bool useHands,
              ref float deepest, ref Vector3 push, ref Vector3 point, ref Kind kind)
    {
        if (!useUsu && !useTable && !useHands) return;
        foreach (var b in balls)
        {
            Vector3 c = pos + rot * new Vector3(b.x, b.y, b.z);
            float r = b.w;
            if (useUsu)
            {
                float d = UsuDistance(c, out Vector3 n);
                if (r - d > deepest) { deepest = r - d; push = n; point = c; kind = Kind.Usu; }
            }
            if (useTable)
            {
                float d = TableDistance(c, out Vector3 n);
                if (r - d > deepest) { deepest = r - d; push = n; point = c; kind = Kind.Table; }
            }
            if (useHands)
                foreach (var h in handBalls)
                {
                    Vector3 dv = c - new Vector3(h.x, h.y, h.z);
                    float dist = dv.magnitude, pen = r + h.w - dist;
                    if (pen > deepest && dist > 1e-6f) { deepest = pen; push = dv / dist; point = c; kind = Kind.Hand; }
                }
        }
    }

    /// <summary>
    /// Lifts the handle off the usu by rotating the kine about a pivot: the holding hand's grip on the
    /// handle when held, else the centre of the head. Returns true if it had to move.
    /// </summary>
    bool PivotHandle(ref Vector3 pos, ref Quaternion rot, bool hasPivot, Vector3 holder)
    {
        bool moved = false;
        for (int pass = 0; pass < 4; pass++)
        {
            float deepest = 1e-4f;
            Vector3 n = Vector3.zero, ball = Vector3.zero;
            foreach (var b in handleBalls)
            {
                Vector3 c = pos + rot * new Vector3(b.x, b.y, b.z);
                float d = UsuDistance(c, out Vector3 nn);
                if (b.w - d > deepest) { deepest = b.w - d; n = nn; ball = c; }
            }
            if (n.sqrMagnitude < 0.5f) break;
            Vector3 pivot = hasPivot ? ClosestOnHandle(pos, rot, holder) : pos + rot * new Vector3(0f, kine.headLength * 0.5f, 0f);
            Vector3 r = ball - pivot;
            Vector3 axis = Vector3.Cross(r, n);
            float lever = r.magnitude, sin = lever > 1e-4f ? axis.magnitude / lever : 0f;
            if (sin < 0.2f)
            {
                pos += n * (deepest + 1e-4f); // pushing along the handle itself: rotation can't help
            }
            else
            {
                float angle = Mathf.Min((deepest + 1e-4f) / (lever * sin), 0.5f) * Mathf.Rad2Deg;
                Quaternion q = Quaternion.AngleAxis(angle, axis / axis.magnitude);
                pos = pivot + q * (pos - pivot);
                rot = q * rot;
            }
            Touch(Kind.Usu, n, ball);
            moved = true;
        }
        return moved;
    }

    void Touch(Kind kind, Vector3 normal, Vector3 point)
    {
        stepTouched = true;
        lastKind = kind;
        lastNormal = normal;
        lastPoint = point;
    }

    Vector3 ClosestOnHandle(Vector3 pos, Quaternion rot, Vector3 p)
    {
        Vector3 a = pos + rot * handleA, b = pos + rot * handleB, ab = b - a;
        float t = Mathf.Clamp01(Vector3.Dot(p - a, ab) / Mathf.Max(ab.sqrMagnitude, 1e-8f));
        return a + ab * t;
    }

    /// <summary>World position of whatever holds the kine: the holding controller, else the holding hand's palm.</summary>
    bool HolderPoint(out Vector3 world)
    {
        world = default;
        if (grabber != null && zone != null)
        {
            var ctrl = grabber.ControllerHolding(zone);
            Transform anchor = ctrl == OVRInput.Controller.LTouch ? grabber.leftControllerAnchor
                             : ctrl == OVRInput.Controller.RTouch ? grabber.rightControllerAnchor : null;
            if (anchor != null) { world = anchor.position; return true; }
        }
        foreach (var tool in station.handTools)
        {
            if (tool == null) continue;
            var hand = HandFor(tool);
            if (hand == null || !hand.IsTracked) continue;
            if (Palm(hand, out Vector3 palm) && zone != null && zone.Distance(palm) < 0.08f) { world = palm; return true; }
        }
        return false;
    }

    /// <summary>Signed distance (negative inside) from a station-local point to the usu solid, and the outward normal.</summary>
    float UsuDistance(Vector3 c, out Vector3 normal)
    {
        Vector3 rel = c - usuBase;
        float rho = new Vector2(rel.x, rel.z).magnitude;
        Vector2 p = new Vector2(rho, rel.y);
        // inside test over every edge; distance only to the real surfaces (not the axis or the buried bottom)
        bool inside = false;
        float best = float.MaxValue;
        Vector2 bestQ = p, bestEdgeN = Vector2.up;
        int n = profile.Length;
        for (int i = 0; i < n; i++)
        {
            Vector2 a = profile[i], b = profile[(i + 1) % n];
            if ((a.y > p.y) != (b.y > p.y) && p.x < a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y)) inside = !inside;
            if (i < 2 || i == n - 1) continue; // buried bottom, buried side below the table, axis
            Vector2 ab = b - a;
            float t = Mathf.Clamp01(Vector2.Dot(p - a, ab) / Mathf.Max(ab.sqrMagnitude, 1e-12f));
            Vector2 q = a + ab * t;
            float d = (p - q).sqrMagnitude;
            if (d < best) { best = d; bestQ = q; bestEdgeN = new Vector2(ab.y, -ab.x).normalized; }
        }
        float dist = Mathf.Sqrt(best);
        Vector2 g = dist > 1e-6f ? (inside ? (bestQ - p) : (p - bestQ)) / dist : bestEdgeN;
        Vector2 radial = rho > 1e-6f ? new Vector2(rel.x, rel.z) / rho : Vector2.right;
        normal = new Vector3(g.x * radial.x, g.y, g.x * radial.y).normalized;
        return inside ? -dist : dist;
    }

    /// <summary>Signed distance from a station-local point to the table (a slab under y = 0), and the outward normal.</summary>
    float TableDistance(Vector3 c, out Vector3 normal)
    {
        Vector3 lo = new Vector3(usuBase.x - tableHalfWidth, -1f, tableNear);
        Vector3 hi = new Vector3(usuBase.x + tableHalfWidth, 0f, usuBase.z + tableFar);
        Vector3 centre = (lo + hi) * 0.5f, half = (hi - lo) * 0.5f;
        Vector3 d = c - centre;
        Vector3 q = new Vector3(Mathf.Abs(d.x) - half.x, Mathf.Abs(d.y) - half.y, Mathf.Abs(d.z) - half.z);
        Vector3 outside = new Vector3(Mathf.Max(q.x, 0f), Mathf.Max(q.y, 0f), Mathf.Max(q.z, 0f));
        float outLen = outside.magnitude;
        if (outLen > 0f)
        {
            normal = new Vector3(outside.x * Mathf.Sign(d.x), outside.y * Mathf.Sign(d.y), outside.z * Mathf.Sign(d.z)) / outLen;
            return outLen;
        }
        // inside: leave through the nearest face
        if (q.y >= q.x && q.y >= q.z) normal = new Vector3(0f, Mathf.Sign(d.y), 0f);
        else if (q.x >= q.z) normal = new Vector3(Mathf.Sign(d.x), 0f, 0f);
        else normal = new Vector3(0f, 0f, Mathf.Sign(d.z));
        return Mathf.Max(q.x, Mathf.Max(q.y, q.z));
    }

    /// <summary>Collects the free tracked hands' primitives as balls in station space. False if there are none.</summary>
    bool GatherHands(Transform st, bool held)
    {
        handBalls.Clear();
        foreach (var tool in station.handTools)
        {
            if (tool == null || !tool.isActiveAndEnabled || tool.primitives == null) continue;
            var hand = HandFor(tool);
            if (hand == null || !hand.IsTracked) continue; // an untracked hand keeps stale primitives
            if (held && IsHolding(hand)) continue;
            int count = Mathf.Min(tool.numPrimitives, tool.primitives.Length);
            for (int j = 0; j < count; j++)
            {
                var p = tool.primitives[j];
                AddBall(st, p.sphere1, p.radii1);
                AddBall(st, p.sphere2, p.radii2);
                if (p.radii3 > 0f) AddBall(st, p.sphere3, p.radii3);
                else AddBall(st, (p.sphere1 + p.sphere2) * 0.5f, (p.radii1 + p.radii2) * 0.5f); // middle of a finger segment
            }
        }
        return handBalls.Count > 0;
    }

    void AddBall(Transform st, Vector3 world, float r)
    {
        if (r <= 0f) return;
        Vector3 p = st.InverseTransformPoint(world);
        handBalls.Add(new Vector4(p.x, p.y, p.z, r));
    }

    OVRHand HandFor(MatTool tool)
    {
        if (handFor.TryGetValue(tool, out var h)) return h;
        var go = GameObject.Find(tool.handType == MatTool.HandType.LeftHand
            ? "OVRCameraRig/TrackingSpace/LeftHandAnchor/LeftOVRHand"
            : "OVRCameraRig/TrackingSpace/RightHandAnchor/RightOVRHand");
        h = go != null ? go.GetComponent<OVRHand>() : null;
        handFor[tool] = h;
        return h;
    }

    static bool Palm(OVRHand hand, out Vector3 palm)
    {
        palm = default;
        var skeleton = hand.GetComponent<OVRSkeleton>();
        var wrist = HandBoneCompat.Find(skeleton, OVRSkeleton.BoneId.Hand_WristRoot);
        var knuckle = HandBoneCompat.Find(skeleton, OVRSkeleton.BoneId.Hand_Middle1);
        if (wrist == null || knuckle == null) return false;
        palm = Vector3.Lerp(wrist.position, knuckle.position, 0.7f); // MochiGrabber's palm point
        return true;
    }

    bool IsHolding(OVRHand hand)
    {
        if (!Palm(hand, out Vector3 palm)) return true; // can't tell: don't let it block its own kine
        return zone.Distance(palm) < 0.08f;
    }

    void Buzz(float speed)
    {
        if (grabber == null || zone == null) return;
        var ctrl = grabber.ControllerHolding(zone);
        if (ctrl == OVRInput.Controller.None) return;
        buzzing = ctrl;
        buzzUntil = Time.unscaledTime + hapticSeconds;
        OVRInput.SetControllerVibration(0.6f, Mathf.Clamp01(hapticAmplitude * (0.5f + 0.25f * speed)), ctrl);
    }

    void UpdateHaptics()
    {
        if (buzzing == OVRInput.Controller.None || Time.unscaledTime < buzzUntil) return;
        if (zone == null || !zone.warning) OVRInput.SetControllerVibration(0f, 0f, buzzing); // KineSafety owns the buzz while it warns
        buzzing = OVRInput.Controller.None;
    }

    void OnDisable()
    {
        if (buzzing != OVRInput.Controller.None) OVRInput.SetControllerVibration(0f, 0f, buzzing);
        buzzing = OVRInput.Controller.None;
    }
}

/// <summary>
/// Adds KineContact to every KineTool in a loaded scene (unless the scene already has one on it, or the
/// station is in test-bench mode). Define MOCHI_KINE_CONTACT_OFF to switch this off.
/// </summary>
public static class KineContactBootstrap
{
#if !MOCHI_KINE_CONTACT_OFF
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Install()
    {
        SceneManager.sceneLoaded -= OnSceneLoaded;
        SceneManager.sceneLoaded += OnSceneLoaded;
        Attach();
    }

    static void OnSceneLoaded(Scene scene, LoadSceneMode loadMode) => Attach();
#endif

    public static void Attach()
    {
        foreach (var kine in UnityEngine.Object.FindObjectsByType<KineTool>(FindObjectsInactive.Exclude))
        {
            if (kine.GetComponent<KineContact>() != null) continue;
            var station = kine.GetComponentInParent<MochiStation>();
            if (station != null && station.toolSet == MochiStation.ToolSet.TestBench) continue;
            kine.gameObject.AddComponent<KineContact>();
        }
    }
}
