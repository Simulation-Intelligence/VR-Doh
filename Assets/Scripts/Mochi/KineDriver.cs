using UnityEngine;

/// <summary>
/// Moves the kine (mallet) head.
///  - Controller: the pounder holds a Touch controller as the handle; the head sits at the end of it.
///  - AutoPound: pounds the usu on a steady rhythm, so one person can practise kneading alone.
///  - External: something else (network sync, test bench) sets the pose.
/// In the colocated game the pounder's controller pose arrives over the network and is applied in External mode.
/// </summary>
[DefaultExecutionOrder(-60)]
public class KineDriver : MonoBehaviour
{
    public enum Mode { Controller, AutoPound, External }
    [Tooltip("External = picked up and swung by hand (MochiGrabber) or driven over the network.")]
    public Mode mode = Mode.External;
    public MochiStation station;

    [Header("Controller")]
    public Transform controllerAnchor;
    [Tooltip("Distance from the grip to the centre of the head along the controller's forward axis, m.")]
    public float handleLength = 0.55f;

    [Header("Auto pound")]
    [Tooltip("Seconds between strikes. Traditional mochitsuki runs about one strike every 1-2 s while the kneader turns the mochi.")]
    public float period = 2.5f;
    [Tooltip("Head speed at impact, m/s.")]
    public float impactSpeed = 4f;
    [Tooltip("Lowest height of the face above the dough floor, m.")]
    public float bottomHeight = 0.025f;
    public float raisedHeight = 0.40f;
    public float liftSpeed = 1.5f;
    public float dwell = 0.05f;
    [Tooltip("Horizontal offset of the strike point from the usu centre (station space), m.")]
    public Vector2 strikeOffset = Vector2.zero;

    [Header("Rest")]
    [Tooltip("Where the kine stands (on its face, handle toward the player) until someone picks it up. Station space, m.")]
    public Vector3 restPosition = new Vector3(0.32f, 0f, -0.02f);
    public float restYaw = 0f;

    float phaseTime;

    void Start()
    {
        var zone = GetComponent<GrabZone>();
        if (zone != null) zone.Grabbed += _ => mode = Mode.External;   // a hand takes over from auto-pound
        if (mode == Mode.External) Rest();
    }

    [ContextMenu("Put Kine Down")]
    public void Rest()
    {
        if (station == null) return;
        transform.SetPositionAndRotation(station.transform.TransformPoint(restPosition),
                                         station.transform.rotation * Quaternion.Euler(0f, restYaw, 0f));
    }

    void Update()
    {
        switch (mode)
        {
            case Mode.Controller:
                if (controllerAnchor != null)
                {
                    // Head axis perpendicular to the handle: face normal = -controller.up at the head.
                    Quaternion rot = controllerAnchor.rotation;
                    Vector3 headCentre = controllerAnchor.position + controllerAnchor.forward * handleLength;
                    var kine = GetComponent<KineTool>();
                    float half = kine != null ? kine.headLength * 0.5f : 0.1f;
                    transform.SetPositionAndRotation(headCentre - (rot * Vector3.up) * half, rot);
                }
                break;
            case Mode.AutoPound:
                if (station == null) break;
                float dt = station.mochi != null && station.mochi.LastFrameSimTime > 0f ? station.mochi.LastFrameSimTime : Time.deltaTime;
                phaseTime = (phaseTime + dt) % Mathf.Max(period, 0.5f);
                float y = AutoPoundHeight(phaseTime);
                Vector3 local = new Vector3(strikeOffset.x, station.FloorY + y, strikeOffset.y);
                transform.SetPositionAndRotation(station.transform.TransformPoint(local), station.transform.rotation);
                break;
        }
    }

    /// <summary>Face height above the floor at time t into a strike cycle: wait raised, fall, dwell, lift.</summary>
    public float AutoPoundHeight(float t)
    {
        float fall = (raisedHeight - bottomHeight) / Mathf.Max(impactSpeed, 0.1f);
        float lift = (raisedHeight - bottomHeight) / Mathf.Max(liftSpeed, 0.1f);
        float wait = Mathf.Max(period - fall - dwell - lift, 0f);
        if (t < wait) return raisedHeight;
        t -= wait;
        if (t < fall) return raisedHeight - impactSpeed * t;
        t -= fall;
        if (t < dwell) return bottomHeight;
        t -= dwell;
        return Mathf.Min(bottomHeight + liftSpeed * t, raisedHeight);
    }
}
