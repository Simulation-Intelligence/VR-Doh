using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Lets tracked hands (index pinch) and Touch controllers (grip) pick up GrabZones:
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
    public float gripOn = 0.6f, gripOff = 0.35f;
    public Transform leftControllerAnchor, rightControllerAnchor;

    class Grabber
    {
        public string name;
        public OVRHand hand; public OVRSkeleton skeleton;
        public Transform controller; public OVRInput.Controller controllerId;
        public bool active; public Vector3 pos; public Quaternion rot; public float strength;
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
        foreach (var z in zones) if (z != null) z.SetHover(z.Held || Nearest(z));

        foreach (var g in grabbers)
        {
            bool pressing = g.held != null ? g.strength > (g.hand != null ? pinchOff : gripOff)
                                           : g.strength > (g.hand != null ? pinchOn : gripOn);
            if (!g.active) pressing = false;

            if (g.held == null && pressing)
            {
                GrabZone best = null; float bestD = grabReach;
                foreach (var z in zones)
                {
                    if (z == null || z.Held || z.moveTarget == null) continue;
                    float d = z.Distance(g.pos);
                    if (d < bestD) { bestD = d; best = z; }
                }
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
        foreach (var g in grabbers) if (g.active && g.held == null && z.Distance(g.pos) < grabReach) return true;
        return false;
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
            g.pos = (thumb.position + index.position) * 0.5f;
            g.rot = wrist.rotation;
            g.strength = g.hand.GetFingerPinchStrength(OVRHand.HandFinger.Index);
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
