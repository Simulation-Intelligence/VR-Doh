using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Lets tracked hands (closing the hand around it, or an index pinch) and Touch controllers (grip) pick up GrabZones:
/// the usu rim moves the whole station, the kine handle lets you swing the mallet.
/// Runs before the kine/tools/simulation so they all see this frame's poses.
/// </summary>
[DefaultExecutionOrder(-100)]
public class MochiGrabber : MonoBehaviour
{
    public List<GrabZone> zones = new();
    [Tooltip("How close (m) the pinch point or controller must be to a zone to grab it.")]
    public float grabReach = 0.03f;
    [Tooltip("Pinch strength to start / keep a hand grab (hysteresis avoids flicker).")]
    public float pinchOn = 0.8f, pinchOff = 0.5f;
    [Tooltip("How closed the hand must be (0 open, 1 fist) to start / keep a grab by wrapping the hand around a zone.")]
    public float fistOn = 0.6f, fistOff = 0.3f;
    [Tooltip("How close (m) the palm must be to a zone to grab it with a closed hand. A hand wrapped around a handle has its palm a few cm from the axis.")]
    public float palmReach = 0.05f;
    public float gripOn = 0.6f, gripOff = 0.35f;
    public Transform leftControllerAnchor, rightControllerAnchor;

    class Grabber
    {
        public string name;
        public OVRHand hand; public OVRSkeleton skeleton;
        public Transform controller; public OVRInput.Controller controllerId;
        public bool active; public Vector3 pos; public Quaternion rot; public float strength;
        // Hands only: the pinch point, the palm centre, how closed the hand is, and which of the two grabs is in use.
        public Vector3 pinchPos, palmPos; public float pinch, fist; public bool byPalm;
        public GrabZone held; public Vector3 offsetPos; public Quaternion offsetRot;
    }
    readonly List<Grabber> grabbers = new();

    void Start()
    {
        foreach (var h in FindObjectsByType<OVRHand>(FindObjectsInactive.Exclude))
        {
            bool left = h.name.ToLower().Contains("left");
            grabbers.Add(new Grabber { name = left ? "LeftHand" : "RightHand", hand = h, skeleton = h.GetComponent<OVRSkeleton>() });
        }
        if (leftControllerAnchor != null) grabbers.Add(new Grabber { name = "LeftController", controller = leftControllerAnchor, controllerId = OVRInput.Controller.LTouch });
        if (rightControllerAnchor != null) grabbers.Add(new Grabber { name = "RightController", controller = rightControllerAnchor, controllerId = OVRInput.Controller.RTouch });
    }

    void Update()
    {
        foreach (var g in grabbers) Sample(g);

        // Hover feedback.
        foreach (var z in zones) if (z != null && z.isActiveAndEnabled) z.SetHover(z.Held || Nearest(z));

        foreach (var g in grabbers)
        {
            bool pressing;
            if (g.hand != null)
            {
                // A hand holds on with whichever gesture it grabbed with; it can start a grab with either.
                bool pinching = g.pinch > (g.held != null && !g.byPalm ? pinchOff : pinchOn);
                bool fisting = g.fist > (g.held != null && g.byPalm ? fistOff : fistOn);
                pressing = g.held != null ? (g.byPalm ? fisting : pinching) : (pinching || fisting);
                if (g.held == null && pressing)
                {
                    // Prefer the closed-hand grab when both gestures are made and the palm is on a zone.
                    g.byPalm = fisting && NearestZone(g.palmPos, palmReach, true) != null;
                    if (!g.byPalm && !pinching) pressing = false;
                }
                g.pos = g.byPalm ? g.palmPos : g.pinchPos;
            }
            else
                pressing = g.strength > (g.held != null ? gripOff : gripOn);
            if (!g.active) pressing = false;

            if (g.held == null && pressing)
            {
                bool palmGrab = g.hand != null && g.byPalm;
                GrabZone best = NearestZone(g.pos, palmGrab ? palmReach : grabReach, palmGrab);
                if (best != null)
                {
                    g.held = best;
                    var t = best.moveTarget;
                    g.offsetRot = Quaternion.Inverse(g.rot) * t.rotation;
                    g.offsetPos = Quaternion.Inverse(g.rot) * (t.position - g.pos);
                    best.NotifyGrabbed();
                }
            }
            else if (g.held != null && !pressing)
            {
                var z = g.held; g.held = null; z.NotifyReleased();
            }

            if (g.held != null)
            {
                var t = g.held.moveTarget;
                Quaternion rot = g.rot * g.offsetRot;
                Vector3 pos = g.pos + g.rot * g.offsetPos;
                if (g.held.uprightOnly)
                {
                    // Keep the grab point under the hand while removing pitch/roll from the target.
                    Vector3 fwd = Vector3.ProjectOnPlane(rot * Vector3.forward, Vector3.up);
                    Quaternion upright = fwd.sqrMagnitude > 1e-6f ? Quaternion.LookRotation(fwd, Vector3.up) : t.rotation;
                    Vector3 grabLocal = Quaternion.Inverse(rot) * (g.pos - pos);
                    pos = g.pos - upright * grabLocal;
                    rot = upright;
                }
                t.SetPositionAndRotation(pos, rot);
            }
        }
    }

    /// <summary>
    /// Whoever holds <paramref name="from"/> now holds <paramref name="to"/> instead, keeping to's current pose relative
    /// to the hand (place it first). Used when a picked-up song scroll turns into a mallet. False if nobody holds from.
    /// </summary>
    public bool Handover(GrabZone from, GrabZone to)
    {
        foreach (var g in grabbers)
        {
            if (g.held != from || to == null || to.moveTarget == null) continue;
            from.NotifyReleased();
            g.held = to;
            var t = to.moveTarget;
            g.offsetRot = Quaternion.Inverse(g.rot) * t.rotation;
            g.offsetPos = Quaternion.Inverse(g.rot) * (t.position - g.pos);
            to.NotifyGrabbed();
            return true;
        }
        return false;
    }

    /// <summary>Where the hand or controller holding <paramref name="z"/> grips, and its rotation. False if nobody does.</summary>
    public bool GripPose(GrabZone z, out Vector3 pos, out Quaternion rot)
    {
        foreach (var g in grabbers)
            if (g.held == z) { pos = g.pos; rot = g.rot; return true; }
        pos = default; rot = Quaternion.identity;
        return false;
    }

    /// <summary>Lets go of <paramref name="z"/> if anything holds it (the mallet turning back into a scroll).</summary>
    public void ForceRelease(GrabZone z)
    {
        foreach (var g in grabbers)
            if (g.held == z) { g.held = null; z.NotifyReleased(); }
    }

    /// <summary>Points on tracked hands that are not holding anything (pinch point and wrist), for safety checks.</summary>
    public void GetFreeHandPoints(List<Vector3> points)
    {
        points.Clear();
        foreach (var g in grabbers)
        {
            if (g.hand == null || !g.active || g.held != null) continue;
            points.Add(g.pos);
            var wrist = HandBoneCompat.Find(g.skeleton, OVRSkeleton.BoneId.Hand_WristRoot);
            if (wrist != null) points.Add(wrist.position);
        }
    }

    /// <summary>The controller holding the given zone, or None (hands or not held).</summary>
    public OVRInput.Controller ControllerHolding(GrabZone z)
    {
        foreach (var g in grabbers) if (g.held == z && g.hand == null) return g.controllerId;
        return OVRInput.Controller.None;
    }

    bool Nearest(GrabZone z)
    {
        foreach (var g in grabbers)
        {
            if (!g.active || g.held != null) continue;
            if (g.hand != null ? z.Distance(g.pinchPos) < grabReach || (z.shape == GrabZone.Shape.Segment && z.Distance(g.palmPos) < palmReach)
                               : z.Distance(g.pos) < grabReach) return true;
        }
        return false;
    }

    /// <summary>The free zone nearest to a point, within reach. A closed hand only takes handles (segments),
    /// so a fist kneading near the rim does not drag the usu away; the rim still needs a pinch.</summary>
    GrabZone NearestZone(Vector3 point, float reach, bool handlesOnly = false)
    {
        GrabZone best = null; float bestD = reach;
        foreach (var z in zones)
        {
            if (z == null || z.Held || z.moveTarget == null || !z.isActiveAndEnabled) continue;
            if (handlesOnly && z.shape != GrabZone.Shape.Segment) continue;
            float d = z.Distance(point);
            if (d < bestD) { bestD = d; best = z; }
        }
        return best;
    }

    void Sample(Grabber g)
    {
        if (g.hand != null)
        {
            g.active = g.hand.IsTracked && g.hand.IsDataValid;
            if (!g.active) return;
            var thumb = HandBoneCompat.Find(g.skeleton, OVRSkeleton.BoneId.Hand_ThumbTip);
            var index = HandBoneCompat.Find(g.skeleton, OVRSkeleton.BoneId.Hand_IndexTip);
            var wrist = HandBoneCompat.Find(g.skeleton, OVRSkeleton.BoneId.Hand_WristRoot);
            if (thumb == null || index == null || wrist == null) { g.active = false; return; }
            g.pinchPos = (thumb.position + index.position) * 0.5f;
            g.rot = wrist.rotation;
            g.pinch = g.hand.GetFingerPinchStrength(OVRHand.HandFinger.Index);
            g.fist = MochiSolver.FistAmount(g.skeleton);
            // Palm centre: partway from the wrist to the middle knuckle. A held handle runs through here.
            var knuckle = HandBoneCompat.Find(g.skeleton, OVRSkeleton.BoneId.Hand_Middle1);
            g.palmPos = knuckle != null ? Vector3.Lerp(wrist.position, knuckle.position, 0.7f) : wrist.position;
            if (g.held == null) g.pos = g.pinchPos;
            g.strength = Mathf.Max(g.pinch, g.fist);
        }
        else
        {
            bool connected = (OVRInput.GetConnectedControllers() & g.controllerId) != 0;
            g.active = connected && g.controller != null;
            if (!g.active) return;
            g.pos = g.controller.position;
            g.rot = g.controller.rotation;
            g.strength = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, g.controllerId);
        }
    }
}
