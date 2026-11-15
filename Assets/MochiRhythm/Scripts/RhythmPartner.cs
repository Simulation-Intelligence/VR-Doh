using UnityEngine;

namespace MochiRhythm
{
    /// <summary>
    /// The auto-pounding partner, on the beat. While rhythm mode runs and nobody holds the kine
    /// (KineDriver in External mode, GrabZone not held), this takes the kine from its rest spot, arcs it
    /// over the rim and pounds like KineDriver's AutoPound: the same raised height, impact speed, sink
    /// depth (bottomHeight), dwell and lift speed, with the handle toward the kneader and pitched up over the
    /// rim near the bottom of the stroke (KineDriver.AutoPoundRotation). Instead of a fixed
    /// period, each fall is timed so the face meets the dough's surface exactly on the next note (song
    /// time as heard). Grab the kine and it is yours; stop the game and it goes back to its rest spot.
    /// </summary>
    [DefaultExecutionOrder(-70)] // after MochiGrabber (-100) knows who holds the kine; before the kine tool (-50) and the solver
    public class RhythmPartner : MonoBehaviour
    {
        public RhythmMode mode;
        [Tooltip("Seconds to lift the kine from its rest spot to above the usu.")]
        public float enterTime = 0.5f;
        [Tooltip("Seconds to put it back.")]
        public float leaveTime = 0.5f;

        public bool Driving { get; private set; }

        enum Stage { Off, Enter, Pound, Leave }
        Stage stage = Stage.Off;
        float t, h; // stage clock (s); face height above the dough floor (m)
        Vector3 fromPos;
        Quaternion fromRot;
        double next = double.NaN, lastImpact = double.NegativeInfinity;
        bool fired;
        float impactSurface;

        void Update()
        {
            var d = mode != null ? mode.Driver : null;
            if (d == null || mode.Station == null) { Driving = false; return; }
            Transform station = mode.Station.transform, kine = d.transform;
            float dt = Time.deltaTime, floor = mode.Station.FloorY;
            Vector2 at = d.strikeOffset;
            bool want = mode.WantsPartner;

            if (want && (stage == Stage.Off || stage == Stage.Leave)) Begin(Stage.Enter, kine);
            else if (!want && (stage == Stage.Enter || stage == Stage.Pound))
            {
                if (mode.KineHeld || d.mode != KineDriver.Mode.External) stage = Stage.Off; // someone took it
                else Begin(Stage.Leave, kine);
            }
            Driving = stage == Stage.Enter || stage == Stage.Pound;

            switch (stage)
            {
                case Stage.Enter:
                {
                    t += dt;
                    float k = Smooth(t / enterTime);
                    Vector3 target = station.TransformPoint(new Vector3(at.x, floor + d.raisedHeight, at.y));
                    Vector3 p = Vector3.Lerp(fromPos, target, k) + station.up * (0.12f * Mathf.Sin(Mathf.PI * k)); // arc over the rim
                    kine.SetPositionAndRotation(p, Quaternion.Slerp(fromRot, d.AutoPoundRotation(d.raisedHeight), k));
                    if (t >= enterTime) { stage = Stage.Pound; h = d.raisedHeight; next = double.NaN; fired = false; lastImpact = double.NegativeInfinity; }
                    break;
                }
                case Stage.Pound:
                    Pound(d, station, kine, floor, at, dt);
                    break;
                case Stage.Leave:
                {
                    if (mode.KineHeld || d.mode != KineDriver.Mode.External) { stage = Stage.Off; break; }
                    t += dt;
                    float k = Smooth(t / leaveTime);
                    Vector3 rest = station.TransformPoint(d.restPosition);
                    Quaternion restRot = station.rotation * Quaternion.Euler(0f, d.restYaw, 0f);
                    Vector3 p = Vector3.Lerp(fromPos, rest, k) + station.up * (0.12f * Mathf.Sin(Mathf.PI * k));
                    kine.SetPositionAndRotation(p, Quaternion.Slerp(fromRot, restRot, k));
                    if (t >= leaveTime) { d.Rest(); stage = Stage.Off; }
                    break;
                }
            }
        }

        void Begin(Stage s, Transform kine)
        {
            stage = s;
            t = 0f;
            fromPos = kine.position;
            fromRot = kine.rotation;
        }

        void Pound(KineDriver d, Transform station, Transform kine, float floor, Vector2 at, float dt)
        {
            double now = mode.Music.SongTime();
            if (double.IsInfinity(now) || double.IsNaN(now)) return;
            var game = mode.Game;
            float raised = d.raisedHeight, bottom = d.bottomHeight, dwell = d.dwell;
            float v = Mathf.Max(d.impactSpeed, 0.5f), lift = Mathf.Max(d.liftSpeed, 0.1f);
            float surface = Mathf.Clamp(mode.SurfaceAboveFloor(at.x, at.y), bottom, raised - 0.05f);

            if (double.IsNaN(next) || next > now + 60) { next = game.NextStrike(now); fired = false; }
            if (!fired && !double.IsInfinity(next) && now >= next)
            {
                fired = true;
                lastImpact = next;
                impactSurface = surface;
                mode.OnPartnerStrike(station.TransformPoint(new Vector3(at.x, floor + surface, at.y)));
            }
            float follow = (impactSurface - bottom) / v; // contact to the bottom of the stroke
            double since = now - lastImpact;
            if (fired && since > follow + dwell) { next = game.NextStrike(now); fired = false; }

            float hNext = Mathf.Min(raised, h + lift * dt);                            // rise toward the top
            if (since >= 0 && since < follow) hNext = impactSurface - v * (float)since;  // follow through into the dough
            else if (since >= follow && since < follow + dwell) hNext = bottom;         // dwell at the bottom
            if (!fired && !double.IsInfinity(next))
                hNext = Mathf.Min(hNext, surface + v * (float)(next - now));             // meet the surface on the note
            h = Mathf.Max(hNext, bottom);
            kine.SetPositionAndRotation(station.TransformPoint(new Vector3(at.x, floor + h, at.y)), d.AutoPoundRotation(h));
        }

        static float Smooth(float x)
        {
            x = Mathf.Clamp01(x);
            return x * x * (3f - 2f * x);
        }
    }
}
