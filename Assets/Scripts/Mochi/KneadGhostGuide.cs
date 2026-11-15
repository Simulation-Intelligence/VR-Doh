using System;
using System.Collections.Generic;
using Oculus.Interaction.HandGrab;
using Oculus.Interaction.HandGrab.Visuals;
using Oculus.Interaction.Input;
using UnityEngine;

/// <summary>
/// Shows translucent Interaction SDK ghost hands over the usu, demonstrating the kneader's moves from the
/// reference video (Nakatanidou mochitsuki): fold the far edge over, press it down flat, and cradle the lump.
///
/// The finger shapes come from MediaPipe hand landmarks averaged over the video frames of each move
/// (Resources/MochiKneadGestures.json, made by mochi_reference/hand_poses). At start each one is retargeted
/// onto the ghost hand's own skeleton and stored as an ISDK <see cref="HandPose"/>, the same data a recorded
/// Hand Grab Pose holds, so the poses could also be used for HandGrabPose or pose recognition later.
/// Where the wrist goes (relative to the usu) is authored here, not taken from the video.
///
/// The ghosts hide while a real hand is in the usu, and come back after the hands have been away a while.
/// </summary>
public class KneadGhostGuide : MonoBehaviour
{
    [Serializable] class GestureFile { public string source; public Gesture[] gestures; }
    [Serializable] class Gesture { public string name; public string description; public float[] points; }

    [Serializable]
    public class Step
    {
        public string gesture = "press";
        [Tooltip("Wrist position at the start and end of the step, in station space (m; +Z away from the kneader, origin at the usu bottom centre).")]
        public Vector3 from, to;
        [Tooltip("Direction the fingers point (station space).")]
        public Vector3 fingers = Vector3.forward;
        [Tooltip("Direction the palm faces (station space).")]
        public Vector3 palm = Vector3.down;
        public float seconds = 1f;
        [Tooltip("Show the mirrored left hand as well (x mirrored about the usu axis).")]
        public bool bothHands;
    }

    public MochiStation station;
    [Tooltip("ISDK ghost hands (OpenXRGhostProvider from the Interaction SDK).")]
    public HandGhostProvider ghostProvider;
    public TextAsset gestureFile;
    [Tooltip("Hand used for one-handed steps. The kneader in the reference uses the right hand.")]
    public Handedness hand = Handedness.Right;
    [Tooltip("Seconds to blend from one step's hand shape into the next.")]
    public float blend = 0.25f;
    [Tooltip("Pause at the end of the sequence before it repeats, s.")]
    public float pause = 1.0f;
    public List<Step> steps = new()
    {
        // Reach over the far side, hook the edge, fold it toward the middle.
        new Step { gesture = "fold",   from = new Vector3(0.02f, 0.24f, 0.13f), to = new Vector3(0.02f, 0.17f, 0.02f), fingers = new Vector3(0, -1, 0.35f), palm = new Vector3(0, -0.3f, -1), seconds = 1.0f },
        // Flat hand presses the fold down and slides forward a little.
        new Step { gesture = "press",  from = new Vector3(0.02f, 0.22f, -0.10f), to = new Vector3(0.02f, 0.17f, -0.06f), fingers = new Vector3(0, -0.15f, 1), palm = Vector3.down, seconds = 0.8f },
        new Step { gesture = "fold",   from = new Vector3(0.10f, 0.24f, 0.11f), to = new Vector3(0.05f, 0.17f, 0.01f), fingers = new Vector3(-0.3f, -1, 0.3f), palm = new Vector3(0.3f, -0.3f, -1), seconds = 1.0f },
        new Step { gesture = "press",  from = new Vector3(0.04f, 0.22f, -0.10f), to = new Vector3(0.04f, 0.17f, -0.06f), fingers = new Vector3(-0.1f, -0.15f, 1), palm = Vector3.down, seconds = 0.8f },
        // Both hands palm-up at the sides of the lump, lifting it to turn it over.
        new Step { gesture = "cradle", from = new Vector3(0.13f, 0.10f, -0.02f), to = new Vector3(0.11f, 0.20f, -0.02f), fingers = new Vector3(-1, 0.2f, 0.3f), palm = Vector3.up, seconds = 1.4f, bothHands = true },
    };

    [Header("Hiding")]
    [Tooltip("Hide the ghosts while a real hand is closer than this to the usu centre, m.")]
    public float hideRadius = 0.25f;
    [Tooltip("Seconds the real hands must stay away before the ghosts come back.")]
    public float showAfter = 4f;
    public Transform leftHandAnchor, rightHandAnchor;
    /// <summary>Set while a rhythm song plays: the ghosts would invite hands into the usu just as notes land.</summary>
    public static bool Suppressed;

    /// <summary>Retargeted finger poses by gesture name, for each hand.</summary>
    public readonly Dictionary<string, HandPose> rightPoses = new(), leftPoses = new();

    HandGhost ghostRight, ghostLeft;
    float t, awayFor;
    bool visible = true;

    void Start()
    {
        if (station == null) station = GetComponentInParent<MochiStation>();
        if (gestureFile == null) gestureFile = Resources.Load<TextAsset>("MochiKneadGestures");
        if (ghostProvider == null || gestureFile == null || station == null)
        {
            Debug.LogWarning("KneadGhostGuide: needs a ghost provider, the gesture file and a MochiStation.");
            enabled = false; return;
        }
        if (leftHandAnchor == null || rightHandAnchor == null)
        {
            var rig = FindAnyObjectByType<OVRCameraRig>();
            if (rig != null) { leftHandAnchor ??= rig.leftHandAnchor; rightHandAnchor ??= rig.rightHandAnchor; }
        }
        var file = JsonUtility.FromJson<GestureFile>(gestureFile.text);
        ghostRight = Instantiate(ghostProvider.GetHand(Handedness.Right), transform);
        ghostLeft = Instantiate(ghostProvider.GetHand(Handedness.Left), transform);
        ghostRight.name = "KneadGhost_Right"; ghostLeft.name = "KneadGhost_Left";
        foreach (var g in file.gestures)
        {
            rightPoses[g.name] = Retarget(ghostRight, g.points, Handedness.Right);
            leftPoses[g.name] = Retarget(ghostLeft, g.points, Handedness.Left);
        }
        Debug.Log($"KneadGhostGuide: {file.gestures.Length} gestures retargeted ({string.Join(", ", rightPoses.Keys)}).");
    }

    void Update()
    {
        UpdateVisibility();
        if (!visible || steps.Count == 0) return;

        float total = pause;
        foreach (var s in steps) total += s.seconds;
        t = (t + Time.deltaTime) % total;

        float acc = 0f; int i = 0;
        for (; i < steps.Count; i++) { if (t < acc + steps[i].seconds) break; acc += steps[i].seconds; }
        bool paused = i >= steps.Count;
        if (paused) { i = steps.Count - 1; acc -= steps[i].seconds; }
        var step = steps[i];
        float u = paused ? 1f : Mathf.Clamp01((t - acc) / step.seconds);
        float e = Mathf.SmoothStep(0f, 1f, u);
        var prev = steps[(i + steps.Count - 1) % steps.Count];
        float shapeBlend = blend > 0f ? Mathf.Clamp01((t - acc) / blend) : 1f;

        bool rightActive = step.bothHands || hand == Handedness.Right;
        bool leftActive = step.bothHands || hand == Handedness.Left;
        ghostRight.gameObject.SetActive(rightActive);
        ghostLeft.gameObject.SetActive(leftActive);
        if (rightActive) Pose(ghostRight, rightPoses, prev, step, e, shapeBlend, false);
        if (leftActive) Pose(ghostLeft, leftPoses, prev, step, e, shapeBlend, true);
    }

    void Pose(HandGhost ghost, Dictionary<string, HandPose> poses, Step prev, Step step, float e, float shapeBlend, bool mirror)
    {
        if (!poses.TryGetValue(step.gesture, out var to)) return;
        var from = poses.TryGetValue(prev.gesture, out var p) ? p : to;
        var shape = new HandPose(to);
        HandPose.Lerp(from, to, shapeBlend, ref shape);

        Vector3 m = mirror ? new Vector3(-1, 1, 1) : Vector3.one;
        Vector3 local = Vector3.Scale(Vector3.Lerp(step.from, step.to, e), m);
        Vector3 fingers = station.transform.TransformDirection(Vector3.Scale(step.fingers, m)).normalized;
        Vector3 palm = station.transform.TransformDirection(Vector3.Scale(step.palm, m)).normalized;
        ghost.SetPose(shape, new UnityEngine.Pose(station.transform.TransformPoint(local), WristRotation(ghost, fingers, palm)));
    }

    // Wrist rotation that makes the ghost's fingers point along `fingers` with its palm facing `palm`.
    readonly Dictionary<HandGhost, Quaternion> wristToHand = new();
    Quaternion WristRotation(HandGhost ghost, Vector3 fingers, Vector3 palm)
    {
        palm = Vector3.ProjectOnPlane(palm, fingers).normalized;
        if (palm.sqrMagnitude < 1e-4f) palm = Vector3.ProjectOnPlane(Vector3.down, fingers).normalized;
        return Quaternion.LookRotation(fingers, palm) * wristToHand[ghost];
    }

    void UpdateVisibility()
    {
        Vector3 c = station.transform.TransformPoint(new Vector3(0f, station.FloorY + 0.08f, 0f));
        bool near = Suppressed || (leftHandAnchor != null && Vector3.Distance(leftHandAnchor.position, c) < hideRadius)
                 || (rightHandAnchor != null && Vector3.Distance(rightHandAnchor.position, c) < hideRadius);
        awayFor = near ? 0f : awayFor + Time.deltaTime;
        bool show = !near && (visible || awayFor > showAfter);
        if (show == visible) return;
        visible = show;
        if (!visible) { ghostRight.gameObject.SetActive(false); ghostLeft.gameObject.SetActive(false); }
        else t = 0f;
    }

    // ---- Retargeting -------------------------------------------------------------------------

    // MediaPipe landmark chains: joint landmark -> child landmark, per ISDK OpenXR joint.
    static readonly (HandJointId joint, int from, int to)[] Chain =
    {
        (HandJointId.HandThumb1, 1, 2), (HandJointId.HandThumb2, 2, 3), (HandJointId.HandThumb3, 3, 4),
        (HandJointId.HandIndex0, 0, 5), (HandJointId.HandIndex1, 5, 6), (HandJointId.HandIndex2, 6, 7), (HandJointId.HandIndex3, 7, 8),
        (HandJointId.HandMiddle0, 0, 9), (HandJointId.HandMiddle1, 9, 10), (HandJointId.HandMiddle2, 10, 11), (HandJointId.HandMiddle3, 11, 12),
        (HandJointId.HandRing0, 0, 13), (HandJointId.HandRing1, 13, 14), (HandJointId.HandRing2, 14, 15), (HandJointId.HandRing3, 15, 16),
        (HandJointId.HandPinky0, 0, 17), (HandJointId.HandPinky1, 17, 18), (HandJointId.HandPinky2, 18, 19), (HandJointId.HandPinky3, 19, 20),
    };

    /// <summary>
    /// Bends the ghost's own bones so each points the way the matching landmark bone does, after mapping
    /// the landmark palm frame onto the ghost's palm, then reads the finger rotations back as a HandPose.
    /// </summary>
    HandPose Retarget(HandGhost ghost, float[] pts, Handedness handedness)
    {
        var puppet = ghost.GetComponentInChildren<HandPuppet>(true);
        var maps = new Dictionary<HandJointId, HandJointMap>();
        foreach (var jm in puppet.JointMaps) if (jm != null && jm.transform != null) maps[jm.id] = jm;

        // Rest pose, to restore afterwards.
        var rest = new Dictionary<Transform, Quaternion>();
        foreach (var jm in maps.Values) rest[jm.transform] = jm.transform.localRotation;

        // Bone direction of each joint in its own space at rest (towards its child joint; distal: continue the parent bone).
        var boneLocal = new Dictionary<HandJointId, Vector3>();
        foreach (var c in Chain)
        {
            if (!maps.TryGetValue(c.joint, out var jm)) continue;
            Transform child = null;
            foreach (var c2 in Chain) if (c2.from == c.to && maps.TryGetValue(c2.joint, out var cm)) { child = cm.transform; break; }
            Vector3 dir = child != null ? child.position - jm.transform.position
                        : (jm.transform.childCount > 0 ? jm.transform.GetChild(0).position - jm.transform.position
                                                       : jm.transform.position - jm.transform.parent.position);
            boneLocal[c.joint] = jm.transform.InverseTransformDirection(dir.normalized);
        }

        // Palm frame of the ghost at rest: +X index side, +Y wrist->middle knuckle, +Z out of the palm.
        Transform wrist = puppet.transform;
        Vector3 w = wrist.position;
        Vector3 idx = maps[HandJointId.HandIndex1].transform.position, mid = maps[HandJointId.HandMiddle1].transform.position,
                lit = maps[HandJointId.HandPinky1].transform.position;
        Vector3 up = (mid - w).normalized;
        Vector3 side = Vector3.ProjectOnPlane(idx - lit, up).normalized;
        // Unity is left-handed: a right palm faces -Cross(side, up), a left palm +Cross(side, up).
        Vector3 palmOut = (handedness == Handedness.Right ? -1f : 1f) * Vector3.Cross(side, up);
        float scale = (mid - w).magnitude;
        if (!wristToHand.ContainsKey(ghost))
            wristToHand[ghost] = Quaternion.Inverse(Quaternion.LookRotation(up, palmOut)) * wrist.rotation;

        Vector3 Target(int i)
        {
            float x = pts[i * 3], y = pts[i * 3 + 1], z = pts[i * 3 + 2];
            return w + scale * (x * side + y * up + z * palmOut);
        }

        foreach (var c in Chain)
        {
            if (!maps.TryGetValue(c.joint, out var jm) || !boneLocal.ContainsKey(c.joint)) continue;
            var tr = jm.transform;
            Vector3 cur = tr.TransformDirection(boneLocal[c.joint]);
            // Metacarpals aim from where the ghost's bone starts; the rest copy the landmark bone direction.
            Vector3 tgt = c.from == 0 ? Target(c.to) - tr.position : Target(c.to) - Target(c.from);
            if (tgt.sqrMagnitude < 1e-8f) continue;
            tr.rotation = Quaternion.FromToRotation(cur, tgt.normalized) * tr.rotation;
        }

        var pose = new HandPose(handedness);
        for (int i = 0; i < FingersMetadata.HAND_JOINT_IDS.Length; i++)
        {
            pose.JointRotations[i] = maps.TryGetValue(FingersMetadata.HAND_JOINT_IDS[i], out var jm)
                ? jm.TrackedRotation : Quaternion.identity;
        }
        foreach (var kv in rest) kv.Key.localRotation = kv.Value;
        return pose;
    }
}
