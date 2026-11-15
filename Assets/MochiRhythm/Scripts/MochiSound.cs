using Satie.Core;
using UnityEngine;

namespace MochiRhythm
{
    /// <summary>
    /// The station's sound through Satie. The scene is Satie/scene.satie, compiled into
    /// Assets/Resources/SatieCore by the satie-unity line (Satie/README.md; run node Satie/use-cut-takes.mjs
    /// first, see Satie/take-edits.json). Satie's component starts by itself; this one tells it what happens.
    ///
    /// RhythmMode hands it every kine strike (pound) and every kine knock on the usu or table (rim_knock).
    /// The rest it works out each frame:
    ///  - binds the four hosts. Satie's script lowercases host ids, so its own GameObject.Find by name misses
    ///    Usu, Kine and MochiSolver; the dough is bound to an anchor at the lump, not to the solver's transform
    ///    (a corner of the simulation box);
    ///  - tent_release: the native solver lets go of the dough stuck to the kine once the kine has pulled away
    ///    for kineTack seconds of simulated time (MochiSolver.TackFor, Particles in MochiSolver.compute). This
    ///    counts the same time with the same "pulling" test, starting at each pound;
    ///  - dough_slap: a free hand's palm or index tip crossing down into the dough surface fast;
    ///  - knead_speed: the fastest free hand touching the dough, smoothed;
    ///  - the scene state: rhythm while a song game runs (the ambience is quieter under the song), free_play otherwise.
    /// kine_speed (the swish loop) is not published unless publishKineSpeed is on, so the swish stays silent.
    ///
    /// When Satie is not running (no compiled scene, or a build target without its native library), Strike and
    /// Knock return false and RhythmMode plays its own mallet clips instead.
    /// </summary>
    [DefaultExecutionOrder(60)] // after RhythmMode (50): the hands and the dough surface are this frame's
    public class MochiSound : MonoBehaviour
    {
        public RhythmMode mode;

        [Header("Dough contact")]
        [Tooltip("A free hand moving down into the dough faster than this (m/s) is a slap.")]
        public float slapSpeed = 0.6f;
        [Tooltip("Seconds before the same hand can slap again.")]
        public float slapCooldown = 0.2f;
        [Tooltip("A hand counts as touching the dough within this height above its surface (m).")]
        public float touchBand = 0.02f;
        [Tooltip("Smoothing time of knead_speed (s).")]
        public float kneadSmoothing = 0.1f;

        [Header("Dough release (native solver only)")]
        [Tooltip("Give up on a release that has not happened this long after a pound (s), e.g. a kine left resting in the dough.")]
        public float releaseTimeout = 3f;

        [Header("Swing")]
        [Tooltip("Publish kine_speed so the swish loop follows the kine. Off by default: Satie's swish take is several separate whooshes, which would play at random moments of a swing.")]
        public bool publishKineSpeed = false;

        /// <summary>Satie's component is up and has a compiled scene.</summary>
        public static bool Live => SatieCoreBehaviour.Instance != null && SatieCoreBehaviour.Instance.Session != null;

        const float KneadMax = 1.5f, KineMax = 8f, StrikeMax = 8f;

        class HandTrack { public bool has; public Vector3 palm, tip; public float lastSlap = -10f; }
        readonly HandTrack[] tracks = { new HandTrack(), new HandTrack() };

        SatieCoreBehaviour boundTo;
        Transform doughAnchor;
        string sentState;
        float knead, kneadSent = -1f, kineSpeed, kineSent = -1f;
        bool bonded, haveFace, haveHead;
        float pullTime, bondAge;
        Vector3 lastFace, lastHead;

        // ---- from RhythmMode ----

        /// <summary>A kine strike into the dough at <paramref name="at"/> (world), moving down at <paramref name="speed"/> m/s.</summary>
        public bool Strike(Vector3 at, float speed)
        {
            if (!Live) return false;
            Bind();
            SatieCore.Signal("strike_speed", Mathf.Clamp(speed, 0f, StrikeMax)); // read by the pound when it fires
            SatieCore.Fire("pound", at);
            bonded = mode != null && mode.Station != null && mode.Station.solver != null; // the Taichi solver has no tack
            pullTime = 0f;
            bondAge = 0f;
            return true;
        }

        /// <summary>The kine knocked the usu or the table at <paramref name="at"/>, approaching at <paramref name="speed"/> m/s.</summary>
        public bool Knock(Vector3 at, float speed)
        {
            if (!Live) return false;
            Bind();
            SatieCore.Signal("strike_speed", Mathf.Clamp(speed, 0f, StrikeMax));
            SatieCore.Fire("rim_knock", at);
            return true;
        }

        // ---- every frame ----

        void Update()
        {
            if (mode == null || mode.Station == null || !Live) return;
            Bind();
            SendState();
            if (mode.Suspended) { SendKnead(0f, true); bonded = false; haveFace = false; return; }
            float dt = Time.deltaTime;
            if (dt <= 0f) return;
            TrackRelease();
            TrackHands(dt);
            if (publishKineSpeed) TrackKine(dt);
        }

        void Bind()
        {
            var satie = SatieCoreBehaviour.Instance;
            if (satie == null || satie == boundTo || mode == null || mode.Station == null) return;
            boundTo = satie;
            var station = mode.Station;
            if (mode.Usu != null) SatieCore.Bind("usu", mode.Usu.transform);
            if (mode.Kine != null) SatieCore.Bind("kine", mode.Kine.transform);
            if (doughAnchor == null)
            {
                doughAnchor = new GameObject("Satie dough anchor").transform;
                doughAnchor.SetParent(station.transform, false); // rides along when the station is placed on a table
            }
            doughAnchor.localPosition = LumpCentreLocal;
            SatieCore.Bind("mochisolver", doughAnchor);
            var bamboo = GameObject.Find("MR_Bamboo");
            if (bamboo != null)
            {
                var anchor = bamboo.transform.Find("Satie anchor");
                if (anchor == null)
                {
                    anchor = new GameObject("Satie anchor").transform;
                    anchor.SetParent(bamboo.transform, false);
                    anchor.localPosition = new Vector3(0f, 3.75f, 0f); // mid-height of the grove, as in the manifest
                }
                SatieCore.Bind("mr_bamboo", anchor);
            }
        }

        /// <summary>Middle of the lump, station space: on the usu axis, 4 cm above the dough floor (the solver's LumpCentre).</summary>
        Vector3 LumpCentreLocal => new Vector3(mode.Axis.x, mode.Station.FloorY + 0.04f, mode.Axis.y);

        void SendState()
        {
            string state = mode.Game != null && mode.Game.Active ? "rhythm" : "free_play";
            if (state == sentState) return;
            if (SatieCore.State(state)) sentState = state;
        }

        // The native solver's release: bonds age only while the kine pulls (faster than pullSpeed, not toward the
        // lump), judged with the kine's displacement over the simulated frame time, and let go after kineTack.
        void TrackRelease()
        {
            var kine = mode.Kine;
            if (kine == null) { haveFace = false; return; }
            Vector3 face = kine.transform.position;
            var solver = mode.Station.solver;
            if (bonded && haveFace && solver != null)
            {
                float simDt = mode.Station.Sim != null ? mode.Station.Sim.LastFrameSimTime : 0f;
                bondAge += Time.deltaTime;
                if (simDt > 0f)
                {
                    Vector3 v = (face - lastFace) / simDt;
                    Vector3 toLump = mode.Station.transform.TransformPoint(LumpCentreLocal) - face;
                    float speed = v.magnitude;
                    bool pulling = speed > solver.pullSpeed && Vector3.Dot(v, toLump) < 0.3f * speed * toLump.magnitude;
                    if (pulling) pullTime += simDt;
                    if (pullTime >= solver.kineTack)
                    {
                        bonded = false;
                        SatieCore.Fire("tent_release", face);
                    }
                }
                if (bondAge > releaseTimeout) bonded = false;
            }
            lastFace = face;
            haveFace = true;
        }

        void TrackHands(float dt)
        {
            var hands = mode.Hands.Hands;
            float fastest = 0f;
            Transform st = mode.Station.transform;
            for (int i = 0; i < tracks.Length; i++)
            {
                var tr = tracks[i];
                if (i >= hands.Count || !hands[i].tracked || hands[i].holding) { tr.has = false; continue; }
                Vector3 palm = st.InverseTransformPoint(hands[i].palm), tip = st.InverseTransformPoint(hands[i].indexTip);
                if (tr.has)
                {
                    fastest = Mathf.Max(fastest, Touch(palm, tr.palm, hands[i].palm, tr, dt), Touch(tip, tr.tip, hands[i].indexTip, tr, dt));
                }
                tr.palm = palm;
                tr.tip = tip;
                tr.has = true;
            }
            knead += (fastest - knead) * (1f - Mathf.Exp(-dt / Mathf.Max(kneadSmoothing, 1e-3f)));
            SendKnead(knead, false);
        }

        /// <summary>One hand point this frame (station space, and world for the event): fires a slap when it crosses
        /// down into the dough fast, and returns its speed while it touches the dough (0 otherwise).</summary>
        float Touch(Vector3 p, Vector3 prev, Vector3 world, HandTrack tr, float dt)
        {
            if (!InBowl(p)) return 0f;
            float surface = mode.SurfaceY(p.x, p.z, 0.03f);
            float down = (prev.y - p.y) / dt;
            if (prev.y > surface && p.y <= surface && down > slapSpeed && Time.time - tr.lastSlap > slapCooldown)
            {
                tr.lastSlap = Time.time;
                SatieCore.Fire("dough_slap", world);
            }
            return p.y < surface + touchBand ? (p - prev).magnitude / dt : 0f;
        }

        bool InBowl(Vector3 p)
        {
            float r = Vector2.Distance(new Vector2(p.x, p.z), mode.Axis);
            return r < mode.BowlRadius + 0.01f && p.y > mode.Station.FloorY - 0.02f && p.y < mode.RimY + 0.05f;
        }

        void SendKnead(float value, bool now)
        {
            value = Mathf.Clamp(value, 0f, KneadMax);
            if (value == kneadSent) return;
            if (!now && Mathf.Abs(value - kneadSent) < 0.01f * KneadMax && !(value == 0f && kneadSent != 0f)) return;
            if (SatieCore.Signal("knead_speed", value)) kneadSent = value;
        }

        void TrackKine(float dt)
        {
            var kine = mode.Kine;
            if (kine == null) return;
            Vector3 head = kine.transform.TransformPoint(new Vector3(0f, kine.headLength * 0.5f, 0f));
            float target = haveHead ? Mathf.Min((head - lastHead).magnitude / dt, KineMax) : 0f;
            lastHead = head;
            haveHead = true;
            kineSpeed += (target - kineSpeed) * (1f - Mathf.Exp(-dt / 0.05f));
            if (Mathf.Abs(kineSpeed - kineSent) < 0.01f * KineMax && !(kineSpeed < 0.01f && kineSent != 0f)) return;
            float v = kineSpeed < 0.01f ? 0f : kineSpeed;
            if (SatieCore.Signal("kine_speed", v)) kineSent = v;
        }

        void OnDestroy()
        {
            if (doughAnchor != null) Destroy(doughAnchor.gameObject);
        }
    }
}
