using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Colocated play puts the pounder's real arm swinging toward the kneader's real hands.
/// While the kine is held, warn (red tint + controller haptics) when its head comes within
/// <see cref="warnDistance"/> of any tracked hand that is not holding it. In the networked
/// game, remote players' hand points are added through <see cref="extraHandPoints"/>.
/// </summary>
[DefaultExecutionOrder(-40)]
public class KineSafety : MonoBehaviour
{
    public MochiGrabber grabber;
    public GrabZone zone;
    public KineTool kine;
    [Tooltip("Distance from the kine head surface to a free hand that triggers the warning, m.")]
    public float warnDistance = 0.30f;
    [Tooltip("Only warn when the head is moving faster than this (m/s), so resting the kine near a hand is fine.")]
    public float minHeadSpeed = 0.5f;
    [Range(0, 1)] public float hapticAmplitude = 0.8f;
    /// <summary>Remote kneader's hand points (world space), filled by the network layer.</summary>
    public readonly List<Vector3> extraHandPoints = new();

    public bool Warning { get; private set; }
    readonly List<Vector3> points = new();
    Vector3 lastHead; bool hasLast;
    OVRInput.Controller buzzing = OVRInput.Controller.None;

    void Reset()
    {
        kine = GetComponent<KineTool>();
        zone = GetComponent<GrabZone>();
        grabber = FindAnyObjectByType<MochiGrabber>();
    }

    void Update()
    {
        if (kine == null) kine = GetComponent<KineTool>();
        if (zone == null) zone = GetComponent<GrabZone>();
        if (grabber == null) grabber = FindAnyObjectByType<MochiGrabber>();

        // Head centre: the face is at the origin and the head extends along +Y.
        Vector3 head = transform.TransformPoint(new Vector3(0f, kine != null ? kine.headLength * 0.5f : 0.1f, 0f));
        float headRadius = kine != null ? Mathf.Max(kine.headRadius, kine.headLength * 0.5f) : 0.1f;
        float speed = hasLast && Time.deltaTime > 0f ? (head - lastHead).magnitude / Time.deltaTime : 0f;
        lastHead = head; hasLast = true;

        bool warn = false;
        if (zone != null && zone.Held && grabber != null && speed >= minHeadSpeed)
        {
            grabber.GetFreeHandPoints(points);
            points.AddRange(extraHandPoints);
            foreach (var p in points)
                if ((p - head).magnitude - headRadius < warnDistance) { warn = true; break; }
        }
        Warning = warn;
        if (zone != null) zone.warning = warn;

        var ctrl = warn && grabber != null ? grabber.ControllerHolding(zone) : OVRInput.Controller.None;
        if (ctrl != buzzing)
        {
            if (buzzing != OVRInput.Controller.None) OVRInput.SetControllerVibration(0f, 0f, buzzing);
            buzzing = ctrl;
        }
        if (buzzing != OVRInput.Controller.None) OVRInput.SetControllerVibration(0.5f, hapticAmplitude, buzzing);
    }

    void OnDisable()
    {
        if (buzzing != OVRInput.Controller.None) OVRInput.SetControllerVibration(0f, 0f, buzzing);
        buzzing = OVRInput.Controller.None;
        if (zone != null) zone.warning = false;
    }
}
