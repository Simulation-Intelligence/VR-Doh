using System.Linq;
using UnityEngine;

/// <summary>
/// Moves the kine (mallet) head.
///  - Controller: the pounder holds a Touch controller as the handle; the head sits at the end of it.
///  - AutoPound: pounds the usu on a steady rhythm, so one person can practise kneading alone. Near the bottom
///    of the stroke the handle pitches up just enough to clear the usu rim, as a real kine tips over the rim
///    (the handle sits level with the middle of the head, so a level handle would pass through it).
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

    [Header("Handle over the rim")]
    [Tooltip("Pitch the handle up near the bottom of the stroke so it clears the usu rim instead of passing through it. Off = the level handle of the original auto-pound.")]
    public bool tiltOverRim = true;
    [Tooltip("Handle radius, m (HandleVisual is 3.5 cm across).")]
    public float handleRadius = 0.0175f;
    [Tooltip("Gap kept between the handle and the top of the rim, m.")]
    public float rimClearance = 0.005f;
    [Tooltip("Most the handle tilts, degrees.")]
    public float maxTilt = 40f;

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
                float dt = station.Sim != null && station.Sim.LastFrameSimTime > 0f ? station.Sim.LastFrameSimTime : Time.deltaTime;
                phaseTime = (phaseTime + dt) % Mathf.Max(period, 0.5f);
                float y = AutoPoundHeight(phaseTime);
                Vector3 local = new Vector3(strikeOffset.x, station.FloorY + y, strikeOffset.y);
                transform.SetPositionAndRotation(station.transform.TransformPoint(local), AutoPoundRotation(y));
                break;
        }
    }

    /// <summary>
    /// Rotation of an auto-pounding kine whose face is h above the dough floor at strikeOffset: the station's
    /// rotation (handle level, toward the kneader), pitched up near the bottom of the stroke just enough for
    /// the handle to clear the usu rim.
    /// </summary>
    public Quaternion AutoPoundRotation(float h)
    {
        Quaternion level = station != null ? station.transform.rotation : transform.rotation;
        return tiltOverRim ? level * Quaternion.Euler(RimTilt(h), 0f, 0f) : level; // +X pitch raises the handle (local -Z) end
    }

    UsuTool usuCache;

    /// <summary>
    /// Smallest pitch (degrees, handle end up) at which the handle clears the top of the usu's rim (UsuTool's
    /// visual lip, 2 cm above its collision depth) with the face h above the dough floor at strikeOffset.
    /// 0 while the head is high enough for a level handle.
    /// </summary>
    public float RimTilt(float h)
    {
        if (station == null) return 0f;
        if (usuCache == null) usuCache = station.sharedTools.OfType<UsuTool>().FirstOrDefault();
        if (usuCache == null) return 0f;
        var usu = usuCache;
        Vector3 usuBase = station.transform.InverseTransformPoint(usu.transform.position);
        float need = usuBase.y + usu.baseThickness + usu.depth + 0.02f + handleRadius + rimClearance;
        float rhoIn = usu.innerRadiusTop - handleRadius; // the handle starts to pass over the rim here
        Vector2 at = strikeOffset - new Vector2(usuBase.x, usuBase.z);
        if (Mathf.Abs(at.x) >= rhoIn) return 0f;
        float zCross = -Mathf.Sqrt(rhoIn * rhoIn - at.x * at.x); // the handle runs toward -Z (the kneader)
        var zone = GetComponent<GrabZone>();
        float handleY = zone != null ? zone.segmentA.y : 0.1f;  // handle height above the face
        float faceY = station.FloorY + h;
        // height of the handle axis where it crosses the rim's inner edge, minus what it needs, at a pitch in degrees
        float Clear(float deg)
        {
            float a = deg * Mathf.Deg2Rad, c = Mathf.Cos(a), s = Mathf.Sin(a);
            float u = (at.y + handleY * s - zCross) / Mathf.Max(c, 1e-3f); // distance along the handle from the head axis
            return faceY + handleY * c + u * s - need;
        }
        if (Clear(0f) >= 0f) return 0f;
        if (Clear(maxTilt) < 0f) return maxTilt;
        float lo = 0f, hi = maxTilt;
        for (int i = 0; i < 14; i++)
        {
            float mid = 0.5f * (lo + hi);
            if (Clear(mid) >= 0f) hi = mid; else lo = mid;
        }
        return hi;
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
