using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Collisions between the kine and the other solid things, which the MPM solver does not do (it only collides
/// the dough with tools; tools pass through each other). After whatever moved the kine this frame (MochiGrabber,
/// KineDriver, the rhythm partner), the kine is kept out of the usu, the table top around it, and free tracked
/// hands while it moves. The geometry is in <see cref="KineCollision"/>, a port of the three.js scene's
/// src/kineContact.js:
///  - the shapes that collide are the shapes that are drawn: the usu skin (MR_UsuSkin) or UsuTool's lathe, the
///    kine skin (MR_KineSkin: head with its bands, tapered handle, grip wrap, knob) or the KineTool cylinder, each
///    as the exact solid of revolution its mesh is lathed from. At start the drawn meshes' vertices are measured
///    against those shapes and the fit is logged ("[kine] collision shapes ...");
///  - every contact is cleared by the smallest pivot about the holding hand (or push along the handle), so the
///    kine rests on what it hits instead of jumping to another surface;
///  - a frame's motion is marched by the free space around the kine, and once it is blocked the kine stays on the
///    near side: a real hand that carries on through the usu leaves the kine resting on it, and the kine comes
///    back to the hand when the hand moves away.
/// Impacts raise <see cref="Contact"/> and buzz the holding controller. Idle while MochiStrikeRecorder or
/// MochiTestBench runs.
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
    [Tooltip("The handle rests on the rim instead of passing through it; the kine pivots about the hand.")]
    public bool handleHitsUsu = true;
    public bool handleHitsTable = true;
    [Tooltip("Off by default: the handle reaches over the kneader's wrists.")]
    public bool handleHitsHands = false;
    [Tooltip("Handle radius, m, when the kine has no skin (HandleVisual is 3.5 cm across). The skin's own handle is used otherwise.")]
    public float handleRadius = 0.0175f;

    [Header("Table (station space; the usu stands on it at y = 0, +Z away from the kneader)")]
    public float tableHalfWidth = 0.6f;
    public float tableFar = 0.6f;
    [Tooltip("Station z of the table's near edge. NaN: from UsuTablePlacer (footprintRadius + edgeInset in front of the usu), else 0.3 m in front.")]
    public float tableNear = float.NaN;

    [Header("Feel")]
    [Tooltip("Approach speed (m/s) above which a contact counts as an impact (event + haptics).")]
    public float impactSpeed = 0.5f;
    [Range(0f, 1f)] public float hapticAmplitude = 0.6f;
    public float hapticSeconds = 0.06f;

    /// <summary>Impact: world point, approach speed (m/s), what was hit.</summary>
    public event Action<Vector3, float, Kind> Contact;
    public bool InContact { get; private set; }
    /// <summary>The geometry: shapes, contacts and marching.</summary>
    public KineCollision Collision { get; private set; }

    KineTool kine;
    GrabZone zone;
    MochiGrabber grabber;
    KineDriver driver;
    readonly List<Vector4> handBalls = new List<Vector4>();   // station-local centre, radius
    readonly Dictionary<MatTool, OVRHand> handFor = new Dictionary<MatTool, OVRHand>();
    KineCollision.Revolved handle;  // kine-local, for the grip point
    float handleFrom, handleTo;     // the handle's extent along its axis
    Vector3 gripLocal;              // where a hand holds it when nobody is: the middle of the grip
    Vector3 usuBase;                // station-local position of the usu's base centre
    Vector3 prevPos, prevTarget;
    Quaternion prevRot = Quaternion.identity, prevTargetRot = Quaternion.identity;
    bool hasPrev, suspended;
    float nextSuspendCheck, buzzUntil;
    OVRInput.Controller buzzing = OVRInput.Controller.None;

    void Awake()
    {
        kine = GetComponent<KineTool>();
        zone = GetComponent<GrabZone>();
        driver = GetComponent<KineDriver>();
        if (station == null) station = GetComponentInParent<MochiStation>();
        if (station == null) station = FindAnyObjectByType<MochiStation>();
        if (usu == null && station != null) usu = station.sharedTools.OfType<UsuTool>().FirstOrDefault();
        if (usu == null && station != null) usu = station.GetComponentInChildren<UsuTool>(true);
        grabber = FindAnyObjectByType<MochiGrabber>();
    }

    void Start()
    {
        if (station == null || kine == null) return;
        BuildCollision();
    }

    /// <summary>Re-reads the kine's head size, its drawn skin and the GrabZone's handle segment (after a mallet change).</summary>
    public void Rebuild()
    {
        if (kine == null || station == null) return;
        BuildCollision();
    }

    // The collision shapes, from the meshes that are drawn.
    void BuildCollision()
    {
        Transform st = station.transform;
        var c = new KineCollision();

        // the usu
        float ro = 0.205f, rTop = 0.15f, rBot = 0.125f, baseY = 0.03f, depth = 0.13f;
        Vector3 usuAxis = Vector3.up;
        usuBase = Vector3.zero;
        if (usu != null)
        {
            ro = usu.outerRadius; rTop = usu.innerRadiusTop; rBot = usu.innerRadiusBottom; baseY = usu.baseThickness; depth = usu.depth;
            usuBase = st.InverseTransformPoint(usu.transform.position);
            usuAxis = st.InverseTransformDirection(usu.transform.up);
        }
        MeshFilter usuMesh = DrawnMesh(usu != null ? usu.transform : null, "UsuVisual");
        bool usuSkin = usuMesh != null && usuMesh.sharedMesh != null && usuMesh.sharedMesh.name.Contains("MR_UsuSkin");
        c.usu = usuSkin ? KineCollision.UsuSkin(ro, baseY, depth, rTop, rBot, usuBase, usuAxis)
                        : KineCollision.UsuLathe(ro, baseY, depth, rTop, rBot, usuBase, usuAxis);

        // the table
        if (float.IsNaN(tableNear))
        {
            var placer = station.GetComponent<UsuTablePlacer>();
            tableNear = usuBase.z - (placer != null ? placer.footprintRadius + placer.edgeInset : 0.3f);
        }
        c.table = new KineCollision.Box
        {
            lo = new Vector3(usuBase.x - tableHalfWidth, -1f, tableNear),
            hi = new Vector3(usuBase.x + tableHalfWidth, 0f, usuBase.z + tableFar),
        };

        // the kine
        MeshFilter kineMesh = DrawnMesh(transform, "KineSkin");
        bool kineSkin = kineMesh != null && kineMesh.sharedMesh != null && kineMesh.sharedMesh.name.Contains("MR_KineSkin");
        float R = kine.headRadius, L = kine.headLength, f = kine.faceRounding;
        KineCollision.Revolved head;
        float thinnest, gripAt;
        if (kineSkin)
        {
            float H = driver != null ? driver.handleLength : 0.55f; // the skin's handle length is KineDriver's (UsuKineSkinBuilder)
            head = KineCollision.KineSkinHead(R, L, f);
            handle = KineCollision.KineSkinHandle(R, L, H);
            thinnest = KineCollision.KineSkinHandleRadius;
            gripAt = H - 0.07f; // the middle of the pink grip wrap
        }
        else
        {
            Vector3 a = zone != null ? zone.segmentA : new Vector3(0f, L * 0.5f, 0f);
            Vector3 b = zone != null ? zone.segmentB : new Vector3(0f, L * 0.5f, -0.63f);
            head = KineCollision.CylinderHead(R, L, Mathf.Clamp(f, 0.002f, R * 0.5f));
            handle = KineCollision.CylinderHandle(a, b, handleRadius, R);
            thinnest = handleRadius;
            gripAt = (b - a).magnitude - 0.07f;
        }
        handleFrom = handle.poly[0].y;
        handleTo = handle.poly[handle.poly.Length - 1].y;
        gripLocal = handle.origin + handle.axis * gripAt;
        c.parts.Add(new KineCollision.Part { shape = head, usu = hitUsu, table = hitTable, hands = hitHands });
        c.parts.Add(new KineCollision.Part { shape = handle, usu = handleHitsUsu, table = handleHitsTable, hands = handleHitsHands });
        float hr = head.MaxRadius();
        c.ends.AddRange(new[]
        {
            Vector3.zero, new Vector3(0f, L, 0f), // face and top centres
            new Vector3(hr, L * 0.5f, 0f), new Vector3(-hr, L * 0.5f, 0f), new Vector3(0f, L * 0.5f, hr), new Vector3(0f, L * 0.5f, -hr),
            handle.origin + handle.axis * handleTo, // the handle's end
        });
        c.minStep = Mathf.Max(0.005f, thinnest);
        c.Build();
        Collision = c;
        LogFit(usuMesh, usuSkin, kineMesh, kineSkin);
    }

    static MeshFilter DrawnMesh(Transform root, string name)
    {
        if (root == null) return null;
        var t = root.Find(name);
        if (t != null && t.TryGetComponent(out MeshFilter mf)) return mf;
        foreach (var m in root.GetComponentsInChildren<MeshFilter>(true))
        {
            var r = m.GetComponent<MeshRenderer>();
            if (r != null && r.enabled && m.gameObject.activeInHierarchy) return m;
        }
        return null;
    }

    // How far the drawn meshes' vertices are from the collision surfaces (the three.js scene builds its collision
    // from its meshes; here the profiles are copies of the skin builder's, so this catches a skin that changed).
    void LogFit(MeshFilter usuMesh, bool usuSkin, MeshFilter kineMesh, bool kineSkin)
    {
        Transform st = station.transform;
        float usuDev = -1f, kineDev = -1f;
        if (usuMesh != null && usuMesh.sharedMesh != null && usuMesh.sharedMesh.isReadable)
        {
            usuDev = 0f;
            foreach (var v in usuMesh.sharedMesh.vertices)
            {
                Vector3 p = st.InverseTransformPoint(usuMesh.transform.TransformPoint(v));
                if (Vector3.Dot(p - usuBase, Collision.usu.axis) < 0.002f) continue; // the bottom it stands on
                usuDev = Mathf.Max(usuDev, Mathf.Abs(Collision.usu.Sdf(p, out _)));
            }
        }
        if (kineMesh != null && kineMesh.sharedMesh != null && kineMesh.sharedMesh.isReadable)
        {
            kineDev = 0f;
            foreach (var v in kineMesh.sharedMesh.vertices)
            {
                Vector3 p = transform.InverseTransformPoint(kineMesh.transform.TransformPoint(v));
                float d = float.MaxValue;
                foreach (var part in Collision.parts) d = Mathf.Min(d, Mathf.Abs(part.shape.Sdf(p, out _))); // on some part's surface
                kineDev = Mathf.Max(kineDev, d);
            }
        }
        string Mm(float x) => x < 0f ? "not readable" : $"{x * 1000f:F1} mm";
        string msg = $"[kine] collision shapes: usu {(usuSkin ? "MR_UsuSkin" : "UsuTool lathe")} ({Mm(usuDev)} from its drawn vertices), " +
                     $"kine {(kineSkin ? "MR_KineSkin" : "KineTool cylinder")} ({Mm(kineDev)})";
        if (usuDev > 0.003f || kineDev > 0.003f) Debug.LogWarning(msg + ". A drawn mesh no longer matches; update the profiles in KineCollision.cs.");
        else Debug.Log(msg);
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
        if (station == null || kine == null || Collision == null) return;
        if (Time.unscaledTime >= nextSuspendCheck)
        {
            nextSuspendCheck = Time.unscaledTime + 1f;
            suspended = Running<MochiStrikeRecorder>() || Running<MochiTestBench>();
        }
        UpdateHaptics();
        if (suspended) { hasPrev = false; Collision.Reset(); InContact = false; return; }

        Transform st = station.transform;
        float dt = Mathf.Max(Time.deltaTime, 1e-4f);
        Vector3 targetPos = st.InverseTransformPoint(transform.position);
        Quaternion targetRot = Quaternion.Inverse(st.rotation) * transform.rotation;
        bool held = zone != null && zone.Held;
        bool resting = !held && hasPrev && (targetPos - prevTarget).sqrMagnitude < 1e-6f && Quaternion.Angle(targetRot, prevTargetRot) < 0.05f;
        bool hands = (hitHands || handleHitsHands) && !resting && GatherHands(st, held); // a resting kine is not shoved around by hands
        Vector3 pivot = gripLocal;
        if (held && HolderPoint(out Vector3 holder)) pivot = OnHandle(Quaternion.Inverse(targetRot) * (st.InverseTransformPoint(holder) - targetPos));

        bool touching = Collision.Resolve(targetPos, targetRot, pivot, hands ? handBalls : null, out Vector3 pos, out Quaternion rot);
        if (touching) transform.SetPositionAndRotation(st.TransformPoint(pos), st.rotation * rot);
        if (touching && !InContact && hasPrev)
        {
            // how fast the contact point was coming at the surface (the same body point at last frame's resolved pose and this frame's target)
            Vector3 body = Quaternion.Inverse(rot) * (Collision.lastPoint - pos);
            Vector3 before = prevPos + prevRot * body, after = targetPos + targetRot * body;
            float approach = Vector3.Dot(after - before, -Collision.lastNormal) / dt;
            if (approach > impactSpeed)
            {
                Contact?.Invoke(st.TransformPoint(Collision.lastPoint), approach, (Kind)(int)Collision.lastKind);
                Buzz(approach);
            }
        }
        InContact = touching;
        prevPos = pos;
        prevRot = rot;
        prevTarget = targetPos;
        prevTargetRot = targetRot;
        hasPrev = true;
    }

    /// <summary>The kine-local point on the handle's axis nearest to p (kine-local), within the handle.</summary>
    Vector3 OnHandle(Vector3 p)
    {
        float t = Mathf.Clamp(Vector3.Dot(p - handle.origin, handle.axis), handleFrom, handleTo);
        return handle.origin + handle.axis * t;
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
