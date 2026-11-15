using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace MochiRhythm
{
    /// <summary>
    /// Rhythm mode for MochiPound: pound and knead to the beat of a song. Attached at runtime to any
    /// scene with a MochiStation (see <see cref="RhythmModeBootstrap"/>), so no scene is edited. It uses
    /// the team's own pieces: the kine (KineTool + KineDriver + GrabZone, picked up through
    /// MochiGrabber), the usu (UsuTool) and the dough through MochiStation.Sim (IMochiSim), so it works
    /// with either solver.
    ///
    /// This component finds those pieces, detects kine strikes (the KineTool face, its local origin,
    /// reaching the dough surface read back from the simulation while moving down), sounds every strike
    /// and knock (through Satie via MochiSound when Satie runs, else its own mallet clips), tracks
    /// whether the kneader's hands turned the dough or are under the kine, and
    /// builds the song player (RhythmMusic), the game (RhythmGame), the on-beat partner (RhythmPartner)
    /// and the touch panel (RhythmPanel). It stays idle (and mutes) while MochiStrikeRecorder or
    /// MochiTestBench runs, so it does not show up in their frames.
    /// </summary>
    [DefaultExecutionOrder(50)] // after the grabber, the partner and the kine tool moved the kine this frame
    public class RhythmMode : MonoBehaviour
    {
        public MochiStation station;

        [Header("Strikes")]
        [Tooltip("The kine face must be moving down at least this fast (m/s) when it meets the dough to count as a strike.")]
        public float minStrikeSpeed = 0.8f;
        [Tooltip("Lift the face this far (m) above where it last hit before the next strike counts.")]
        public float rearmLift = 0.06f;

        [Header("Partner")]
        [Tooltip("Pound every note automatically while nobody holds the kine (so one person can knead to the song).")]
        public bool partnerEnabled = true;

        [Header("Look")]
        [Tooltip("How far outside the rim (m) the approach rings start.")]
        public float ringReach = 0.35f;

        [Header("Dough readback (asynchronous; each read copies the particle buffer off the GPU)")]
        public float readbackHzPlaying = 8f;
        public float readbackHzIdle = 2f;

        public MochiStation Station => station;
        public RhythmMusic Music { get; private set; }
        public RhythmGame Game { get; private set; }
        public RhythmPartner Partner { get; private set; }
        public RhythmStoryBridge Story { get; private set; }
        /// <summary>Song choice: one scroll per song that turns into its mallet when grabbed (replaces the panel).</summary>
        public SongScrolls Scrolls { get; private set; }
        public RhythmPanel Panel { get; private set; }
        /// <summary>The Satie side of the station's sound (pound, rim_knock, tent_release, dough_slap, knead_speed).</summary>
        public MochiSound Sound { get; private set; }
        public KineDriver Driver { get; private set; }
        public KineTool Kine { get; private set; }
        public GrabZone KineZone { get; private set; }
        public UsuTool Usu { get; private set; }
        public MochiGrabber Grabber { get; private set; }
        public HandPoints Hands { get; } = new HandPoints();
        /// <summary>Fingertips of free hands and the tips of connected controllers this frame, for the buttons.</summary>
        public readonly List<Vector3> PressPoints = new List<Vector3>();
        /// <summary>A development recorder or test bench is running; rhythm mode keeps out of the way.</summary>
        public bool Suspended { get; private set; }

        public bool KineHeld => KineZone != null && KineZone.Held;
        /// <summary>A person swings the kine: held by a hand or controller, or KineDriver in Controller mode.</summary>
        public bool PlayerPounding => KineHeld || (Driver != null && Driver.mode == KineDriver.Mode.Controller);
        public bool WantsPartner => Game != null && Game.Active && partnerEnabled && !Suspended && Driver != null
                                    && Driver.mode == KineDriver.Mode.External && !KineHeld;

        // usu geometry in station space (metres)
        Vector2 axis;          // usu axis (x, z)
        float rimY = 0.16f, bowlRadius = 0.15f, rimRadius = 0.1775f;
        public float RimRadius => rimRadius;
        /// <summary>The usu axis (x, z), the bowl's radius at the rim and the rim height, all in station space (m).</summary>
        public Vector2 Axis => axis;
        public float BowlRadius => bowlRadius;
        public float RimY => rimY;
        public Vector3 RimCenter => station.transform.TransformPoint(new Vector3(axis.x, rimY + 0.008f, axis.y));
        public Vector3 PopupAnchor => station.transform.TransformPoint(new Vector3(axis.x + 0.12f, rimY + 0.22f, axis.y + 0.05f));
        public Vector3 HudAnchor => station.transform.TransformPoint(new Vector3(axis.x - 0.28f, rimY + 0.36f, axis.y + 0.2f));
        /// <summary>Where the panel lies on the table: to the kneader's left (the station faces away from them).</summary>
        public Vector3 PanelLocalPosition => new Vector3(axis.x - 0.36f, 0.045f, axis.y - 0.06f); // tilted 15 deg, so lifted clear of the table

        DoughHeight height;
        AudioSource hitSource, knockSource;
        KineContact contact;
        AudioClip[] hitClips = new AudioClip[0];
        bool armed = true, hasPrev;
        Vector3 prevFace;
        float strikeSurface, bowlTime, nextSuspendCheck;
        const float GuessLump = 0.07f; // lump height above the floor before the first reading (m)

        void Awake()
        {
            if (station == null) station = FindAnyObjectByType<MochiStation>();
            if (station == null) { Debug.LogWarning("[rhythm] no MochiStation in the scene"); enabled = false; return; }

            Driver = station.GetComponentInChildren<KineDriver>(true);
            if (Driver == null) Driver = FindAnyObjectByType<KineDriver>();
            Kine = Driver != null ? Driver.GetComponent<KineTool>() : null;
            if (Kine == null) Kine = station.sharedTools.OfType<KineTool>().FirstOrDefault();
            KineZone = Kine != null ? Kine.GetComponent<GrabZone>() : null;
            Usu = station.sharedTools.OfType<UsuTool>().FirstOrDefault();
            if (Usu == null) Usu = station.GetComponentInChildren<UsuTool>(true);
            Grabber = FindAnyObjectByType<MochiGrabber>();
            ReadUsu();
            height = new DoughHeight(station, axis, bowlRadius + 0.03f);

            var hits = new GameObject("Kine hits");
            hits.transform.SetParent(transform, false);
            hitSource = hits.AddComponent<AudioSource>();
            hitSource.playOnAwake = false;
            hitSource.spatialBlend = 1f;
            hitSource.minDistance = 0.5f;
            hitSource.maxDistance = 15f;
            hitSource.priority = 8; // above Satie's voices, so it is never virtualized (see RhythmMusic)
            var knocks = new GameObject("Kine knocks");
            knocks.transform.SetParent(transform, false);
            knockSource = knocks.AddComponent<AudioSource>();
            knockSource.playOnAwake = false;
            knockSource.spatialBlend = 1f;
            knockSource.minDistance = 0.5f;
            knockSource.maxDistance = 15f;
            knockSource.priority = 8; // above Satie's voices, so it is never virtualized (see RhythmMusic)
            knockSource.pitch = 1.35f; // the same mallet takes, higher and shorter: wood on wood
            hitClips = Resources.LoadAll<AudioClip>("MochiRhythm/Sounds");
            var songs = Resources.LoadAll<AudioClip>("MochiRhythm/Music").OrderBy(c => c.name, StringComparer.OrdinalIgnoreCase).ToList();

            Music = AddInactive<RhythmMusic>("Music", transform, m => m.songs = songs);
            Game = AddInactive<RhythmGame>("Rhythm game", transform, g => g.mode = this);
            Partner = AddInactive<RhythmPartner>("Rhythm partner", transform, p => p.mode = this);
            Story = AddInactive<RhythmStoryBridge>("Story bridge", transform, b => b.mode = this);
            Panel = AddInactive<RhythmPanel>("Rhythm panel", station.transform, p => p.mode = this); // rides along when the station moves
            Panel.transform.localPosition = PanelLocalPosition;
            Panel.transform.localRotation = Quaternion.Euler(-15f, 0f, 0f); // tilted toward the kneader
            Sound = AddInactive<MochiSound>("Sound", transform, s => s.mode = this);
            Scrolls = AddInactive<SongScrolls>("Song scrolls", transform, s => s.mode = this);

            Debug.Log($"[rhythm] rhythm mode on {station.name}: kine {(Kine != null ? Kine.name : "MISSING")}, usu {(Usu != null ? Usu.name : "MISSING")}, " +
                      $"{songs.Count} songs, {hitClips.Length} kine sounds");
        }

        static T AddInactive<T>(string name, Transform parent, Action<T> setup) where T : Component
        {
            var go = new GameObject(name);
            go.SetActive(false); // so Awake runs after setup
            go.transform.SetParent(parent, false);
            var c = go.AddComponent<T>();
            setup(c);
            go.SetActive(true);
            return c;
        }

        void ReadUsu()
        {
            if (Usu == null) return;
            Transform st = station.transform, u = Usu.transform;
            Vector3 a = st.InverseTransformPoint(u.position);
            axis = new Vector2(a.x, a.z);
            rimY = st.InverseTransformPoint(u.TransformPoint(new Vector3(0f, Usu.baseThickness + Usu.depth, 0f))).y;
            bowlRadius = Usu.innerRadiusTop;
            rimRadius = 0.5f * (Usu.innerRadiusTop + Usu.outerRadius);
        }

        void Update()
        {
            if (Time.unscaledTime >= nextSuspendCheck)
            {
                nextSuspendCheck = Time.unscaledTime + 1f;
                bool dev = Running<MochiStrikeRecorder>() || Running<MochiTestBench>();
                if (dev != Suspended) SetSuspended(dev);
            }
            if (contact == null && Kine != null)
            {
                contact = Kine.GetComponent<KineContact>(); // added by KineContactBootstrap
                if (contact != null) contact.Contact += OnKineContact;
            }
            if (Suspended) return;
            float dt = Time.deltaTime;
            Hands.Refresh(KineZone);
            CollectPressPoints();
            height.Tick(Time.unscaledTime, 1f / Mathf.Max(0.5f, Game.Active ? readbackHzPlaying : readbackHzIdle));
            // time a free hand spends turning the dough, counted between strikes (the partner's or a player's)
            if (!Game.Active) bowlTime = 0f;
            else if (AnyFreeHand(InBowl)) bowlTime += dt;
            DetectStrike(dt);
        }

        // Only an enabled recorder or test bench counts: scenes keep a disabled MochiTestBench on the station,
        // and FindAnyObjectByType also returns disabled components.
        static bool Running<T>() where T : Behaviour
        {
            foreach (var o in FindObjectsByType<T>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
                if (o.isActiveAndEnabled) return true;
            return false;
        }

        void SetSuspended(bool on)
        {
            Suspended = on;
            if (on) { Game.StopGame(); Music.Stop(); }
            Panel.gameObject.SetActive(!on);
            Debug.Log(on ? "[rhythm] a recorder or test bench is running: rhythm mode paused" : "[rhythm] rhythm mode back");
        }

        void CollectPressPoints()
        {
            PressPoints.Clear();
            foreach (var h in Hands.Hands) if (h.tracked && !h.holding) PressPoints.Add(h.indexTip);
            if (Grabber == null) return;
            var connected = OVRInput.GetConnectedControllers();
            AddController(Grabber.leftControllerAnchor, (connected & OVRInput.Controller.LTouch) != 0);
            AddController(Grabber.rightControllerAnchor, (connected & OVRInput.Controller.RTouch) != 0);
        }

        void AddController(Transform anchor, bool connected)
        {
            if (anchor == null || !connected) return;
            Vector3 tip = anchor.position + anchor.forward * 0.04f;
            if (KineHeld && KineZone.Distance(anchor.position) < 0.12f) return; // this one is holding the kine
            PressPoints.Add(tip);
        }

        // ---- the dough surface ----

        /// <summary>Station-space height of the dough surface under a disc around (x, z); the bare floor where there is none.</summary>
        public float SurfaceY(float x, float z, float radius)
        {
            float top = height.TopUnder(x, z, radius);
            if (float.IsNegativeInfinity(top)) return station.FloorY + (height.Valid ? 0f : GuessLump);
            return top + 0.005f; // particle centres sit about half a cell under the drawn surface
        }

        /// <summary>Height of the dough surface above the dough floor under the kine face at (x, z), m.</summary>
        public float SurfaceAboveFloor(float x, float z) => SurfaceY(x, z, Kine != null ? Kine.headRadius : 0.06f) - station.FloorY;

        // ---- hands ----

        bool AnyFreeHand(Func<Vector3, bool> test)
        {
            foreach (var h in Hands.Hands)
                if (h.tracked && !h.holding && (test(h.palm) || test(h.indexTip))) return true;
            return false;
        }

        bool InBowl(Vector3 world)
        {
            Vector3 p = station.transform.InverseTransformPoint(world);
            float r = Vector2.Distance(new Vector2(p.x, p.z), axis);
            return r < bowlRadius + 0.01f && p.y > station.FloorY - 0.02f && p.y < rimY + 0.03f;
        }

        /// <summary>A free hand is under the kine face (within the head's radius plus 4 cm, below the top of the head).</summary>
        bool HandsUnder(Vector3 faceWorld)
        {
            if (Kine == null) return false;
            Vector3 f = station.transform.InverseTransformPoint(faceWorld);
            float reach = Kine.headRadius + 0.04f, topOfHead = f.y + Kine.headLength;
            return AnyFreeHand(w =>
            {
                Vector3 p = station.transform.InverseTransformPoint(w);
                return Vector2.Distance(new Vector2(p.x, p.z), new Vector2(f.x, f.z)) < reach && p.y < topOfHead && p.y > station.FloorY - 0.02f;
            });
        }

        // ---- strikes ----

        void DetectStrike(float dt)
        {
            if (Kine == null) return;
            if (Partner.Driving) { hasPrev = false; return; } // the partner reports its own strikes, on the beat
            Vector3 face = station.transform.InverseTransformPoint(Kine.transform.position); // KineTool origin = face centre
            if (!hasPrev) { prevFace = face; hasPrev = true; return; }
            float down = (prevFace.y - face.y) / Mathf.Max(dt, 1e-4f);
            prevFace = face;
            if (Vector2.Distance(new Vector2(face.x, face.z), axis) > bowlRadius)
            {
                if (face.y > rimY + rearmLift) armed = true;
                return;
            }
            float surface = SurfaceY(face.x, face.z, Kine.headRadius);
            if (!armed && face.y > strikeSurface + rearmLift) armed = true;
            if (armed && face.y <= surface && down > minStrikeSpeed)
            {
                armed = false;
                strikeSurface = surface;
                double since = Mathf.Clamp((surface - face.y) / down, 0f, dt); // back-date to the crossing
                PlayHit(Kine.transform.position, down);
                if (PlayerPounding && Game.Active)
                {
                    bool turned = bowlTime >= 0.12f;
                    bowlTime = 0f;
                    Game.PlayerStrike(Music.SongTime() - since, HandsUnder(Kine.transform.position), turned);
                }
            }
        }

        /// <summary>Called by RhythmPartner when its kine meets the dough on a note.</summary>
        public void OnPartnerStrike(Vector3 faceWorld)
        {
            PlayHit(faceWorld, 4f);
            bool turned = bowlTime >= 0.12f;
            bowlTime = 0f;
            Game.PartnerStrike(HandsUnder(faceWorld), turned);
        }

        // the kine knocked the usu or the table (KineContact); hands make no sound
        void OnKineContact(Vector3 at, float speed, KineContact.Kind kind)
        {
            if (Suspended || kind == KineContact.Kind.Hand) return;
            if (Sound != null && Sound.Knock(at, speed)) return; // Satie's rim_knock
            if (hitClips == null || hitClips.Length == 0) return;
            var clip = hitClips[UnityEngine.Random.Range(0, hitClips.Length)];
            if (clip == null) return;
            knockSource.transform.position = at;
            knockSource.PlayOneShot(clip, Mathf.Clamp01(0.2f + 0.15f * speed));
        }

        void OnDestroy()
        {
            if (contact != null) contact.Contact -= OnKineContact;
        }

        void PlayHit(Vector3 at, float speed)
        {
            if (Sound != null && Sound.Strike(at, speed)) return; // Satie's pound
            if (hitClips == null || hitClips.Length == 0) return;
            var clip = hitClips[UnityEngine.Random.Range(0, hitClips.Length)];
            if (clip == null) return;
            hitSource.transform.position = at;
            hitSource.PlayOneShot(clip, Mathf.Clamp01(0.35f + 0.2f * speed));
        }
    }
}
