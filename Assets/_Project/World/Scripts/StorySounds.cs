using System;
using System.Collections.Generic;
using UnityEngine;

namespace MoonRabbit.World
{
    /// <summary>
    /// Story sound effects, synthesized in code (no audio assets): while an intro line rises letter by letter, a few
    /// long hanging chime bars are brushed one after another ("chiririn"), very softly (tubular-bell partials, high,
    /// pentatonic, soft attack, long decay; a few more when the scroll unrolls and when the last line swells);
    /// a shimmer from the Moon when progress rises (higher with progress, pentatonic), a fuller chord at the 25/50/75 %
    /// milestones, a warm swell when the Moon is full, and the ending's flight whoosh, arrival burst and wish twinkles.
    /// All 3D: text sounds come from the text; Moon / Earth sounds come from their direction, a few metres from the
    /// head, so distance attenuation never mutes them (spatialized with the project's spatializer, e.g. Meta XR Audio).
    /// Any slot can be replaced with a real clip (empty = synthesized). Quiet and high by default, to sit above music.
    /// </summary>
    [RequireComponent(typeof(MoonProgress))]
    public class StorySounds : MonoBehaviour
    {
        [Header("On / off and volume")]
        [Tooltip("Master switch. Off by default (Jihyeon): the rhythm game's audio leads; the hooks on MoonProgress stay.")]
        public bool soundsEnabled = false;
        public bool textSounds = true;
        public bool progressSounds = true;
        [Tooltip("Shimmer on every progress step. Turn off when the rhythm game plays its own hit sounds.")]
        public bool strikeSound = true;
        [Range(0f, 1f)] public float textVolume = 0.11f;
        [Tooltip("Chime when the ending text appears (an ending effect: off with the other ending sounds).")]
        public bool endingTextSound = false;
        [Tooltip("Hanji paper rustle when the intro scroll unrolls and rolls up (instead of chimes).")]
        public bool scrollSounds = true;
        [Range(0f, 1f)] public float paperVolume = 0.1f;
        [Tooltip("Rubbed singing-bowl / glass swell under the last intro line's swell.")]
        public bool swellSound = true;
        [Tooltip("Distant fireworks when the mochi reaches the Earth (on its own; the other ending sounds stay off).")]
        public bool endingFirework = false; // off for now (Jihyeon); the sound is kept
        [Range(0f, 1f)] public float fireworkVolume = 0.07f;
        [Tooltip("The fireworks are far away: heard a moment after the flash (s).")]
        public float fireworkDelay = 0.3f;
        [Range(0f, 1f)] public float progressSfxVolume = 0.3f;

        [Header("Clips (empty = synthesized)")]
        public AudioClip introLineClip;
        public AudioClip instructionClip;
        public AudioClip endingClip;
        public AudioClip strikeClip;
        public AudioClip milestoneClip;
        public AudioClip fullMoonClip;
        public AudioClip flightClip;
        public AudioClip arrivalClip;
        public AudioClip wishClip;

        [Header("Placement")]
        [Tooltip("Moon / Earth sounds play this far from the head in their direction (m).")]
        public float directionalDistance = 6f;
        [Range(1, 32)] public int voices = 20; // the ending firework uses up to 16 at once

        [Header("Text chimes (brushed hanging bars)")]
        [Tooltip("Bars brushed while a line rises (min..max, by line length).")]
        public Vector2Int barsPerLine = new Vector2Int(3, 5);
        [Tooltip("Extra bars when the scroll unrolls and when the last line swells.")]
        public int barsScroll = 6, barsSwell = 6;
        [Tooltip("Bars sit this far to either side of the text (m): a little width, all from the scroll.")]
        public float barSpread = 0.35f;

        // pentatonic steps (major): each intro line and each progress step a little higher
        static readonly float[] Penta = { 1f, 9f / 8f, 5f / 4f, 3f / 2f, 5f / 3f, 2f, 9f / 4f, 5f / 2f, 3f };

        MoonProgress progress;
        AudioClip[] bars;
        AudioClip synPaperOpen, synPaperClose, synSwellTone;
        AudioClip[] fireworkPops;
        AudioClip synIntro, synInstruction, synEnding, synStrike, synMilestone, synFull, synFlight, synArrival, synWish;
        readonly List<AudioSource> pool = new List<AudioSource>();
        int next;
        float lastValue;

        void Awake()
        {
            progress = GetComponent<MoonProgress>();
            Synthesize();
            EnsureListener();
            bool spatial = SpatializerLoaded(out string spatializer);
            for (int i = 0; i < voices; i++)
            {
                var go = new GameObject("MR_StorySound_" + i);
                go.transform.SetParent(transform, false);
                var a = go.AddComponent<AudioSource>();
                a.playOnAwake = false;
                a.spatialBlend = 1f;
                a.spatialize = spatial; // HRTF when a spatializer plugin is set, plain 3D panning otherwise
                a.rolloffMode = AudioRolloffMode.Logarithmic;
                a.minDistance = 1.5f;
                a.maxDistance = 40f;
                a.dopplerLevel = 0f;
                a.priority = 96;
                pool.Add(a);
            }
        }

        /// <summary>
        /// True only when the project's spatializer is actually available in this player. The AudioManager may name
        /// one (e.g. "Meta XR Audio") whose package is not installed: a source with spatialize = true then plays silence
        /// on Quest, so we fall back to Unity's own 3D panning.
        /// </summary>
        static bool SpatializerLoaded(out string name)
        {
            name = AudioSettings.GetSpatializerPluginName() ?? "";
            if (string.IsNullOrEmpty(name)) return false;
#if UNITY_EDITOR
            // GetSpatializerPluginNames is Editor-only; players cannot tell whether the named
            // plugin is really present, so they always use Unity's 3D panning (never silent).
            try
            {
                var available = AudioSettings.GetSpatializerPluginNames();
                if (available == null) return false;
                foreach (var a in available) if (a == name) return true;
            }
            catch (Exception) { }
#endif
            return false;
        }

        /// <summary>Adds an AudioListener to the main camera only if the scene has none at all (never a second one).</summary>
        static void EnsureListener()
        {
            if (FindObjectsByType<AudioListener>(FindObjectsInactive.Exclude, FindObjectsSortMode.None).Length > 0) return;
            var cam = Camera.main;
            if (cam == null) return;
            cam.gameObject.AddComponent<AudioListener>();
            Debug.Log("[Moon Rabbit] Audio: no AudioListener in the scene, added one to " + cam.name);
        }

        void LogAudio()
        {
            var listeners = FindObjectsByType<AudioListener>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
            SpatializerLoaded(out string sp);
            string avail = "";
#if UNITY_EDITOR
            try { avail = string.Join("|", AudioSettings.GetSpatializerPluginNames() ?? new string[0]); } catch (Exception) { }
#else
            avail = "(player: not queryable)";
#endif
            var a = pool.Count > 0 ? pool[0] : null;
            Debug.Log($"[Moon Rabbit] Audio: listeners={listeners.Length}{(listeners.Length > 0 ? " on " + listeners[0].name : "")} " +
                      $"spatializer='{sp}' available='{avail}' spatialize={(a != null && a.spatialize)} " +
                      $"listenerPause={AudioListener.pause} listenerVolume={AudioListener.volume:F2} " +
                      $"enabled={soundsEnabled} text={textSounds} progress={progressSounds} textVol={textVolume:F2} sfxVol={progressSfxVolume:F2} " +
                      (a != null ? $"source: mute={a.mute} blend={a.spatialBlend} min={a.minDistance} max={a.maxDistance} prio={a.priority} " : "") +
                      $"sampleRate={AudioSettings.outputSampleRate} speakerMode={AudioSettings.speakerMode}");
        }

        void Start()
        {
            LogAudio();
            progress.onChanged.AddListener(OnProgress);
            progress.onMilestone.AddListener(OnMilestone);
            lastValue = progress.Value;
        }

        // ------------------------------------------------------------------ public API

        /// <summary>
        /// Brushed chime bars while an intro line rises: <paramref name="riseTime"/> = how long its letters take to
        /// rise; the bars follow them (3-5 by length). With a clip in introLineClip / instructionClip that plays instead.
        /// </summary>
        public void PlayIntroLine(Vector3 at, Vector3 right, int lineIndex, int letters, float riseTime)
        {
            if (!soundsEnabled || !textSounds) return;
            if (introLineClip != null) { Play(introLineClip, at, textVolume, 1f, 0f, 1.5f); return; }
            int n = Mathf.Clamp(Mathf.RoundToInt(Mathf.Lerp(barsPerLine.x, barsPerLine.y, Mathf.InverseLerp(15, 70, letters))),
                                barsPerLine.x, barsPerLine.y);
            PlayCascade(at, right, n, Mathf.Max(0.4f, riseTime * 0.9f), 0f, 101 + lineIndex * 17, 1f);
        }

        /// <summary>Soft hanji rustle as the scroll unrolls (about <paramref name="time"/> s).</summary>
        public void PlayScrollUnroll(Vector3 at, Vector3 right, float time)
        {
            if (!soundsEnabled || !scrollSounds) return;
            Play(synPaperOpen, at, paperVolume, Mathf.Clamp(0.7f / Mathf.Max(0.2f, time), 0.8f, 1.25f), 0f, 1.5f); // made for 0.7 s
        }

        /// <summary>The scroll rolling up: a rustle that ends in a soft tap as the roll closes.</summary>
        public void PlayScrollClose(Vector3 at, float time)
        {
            if (!soundsEnabled || !scrollSounds) return;
            Play(synPaperClose, at, paperVolume, Mathf.Clamp(0.9f / Mathf.Max(0.2f, time), 0.8f, 1.25f), 0f, 1.5f); // made for 0.9 s
        }

        /// <summary>The last line's swell: a rubbed singing-bowl / glass tone that blooms and fades with it.</summary>
        public void PlaySwell(Vector3 at, Vector3 right, float swellTime, float delay)
        {
            if (!soundsEnabled || !swellSound) return;
            if (instructionClip != null) { Play(instructionClip, at, textVolume, 1f, delay, 1.5f); return; }
            Play(synSwellTone, at, textVolume, 1f, delay, 1.5f); // blooms to its peak at ~0.95 s, the swell's top
        }

        /// <summary>
        /// Distant fireworks from the Earth's direction, following the burst: 10-16 small dull pops, close together at
        /// first, then thinning out over about 2 s ("ta-ta-tak~"), each a little higher or lower, louder or softer and a
        /// few degrees left or right of the Earth, with a far-away tail; heard a moment after the flash.
        /// </summary>
        public void PlayFireworkFar(Vector3 towardWorld)
        {
            if (!soundsEnabled || !endingFirework || fireworkPops == null) return;
            var cam = MoonStory.MainCamera();
            var head = cam != null ? cam.transform.position : transform.position + Vector3.up * 1.6f;
            var dir = towardWorld - head;
            dir = dir.sqrMagnitude > 1e-4f ? dir.normalized : Vector3.forward;
            foreach (var p in FireworkSchedule(Time.frameCount))
            {
                var d = Quaternion.AngleAxis(p.angle, Vector3.up) * Quaternion.AngleAxis(p.tilt, Vector3.Cross(Vector3.up, dir)) * dir;
                Play(fireworkPops[p.variant], head + d * directionalDistance, fireworkVolume * p.gain, p.pitch,
                     fireworkDelay + p.time, directionalDistance);
            }
        }

        /// <summary>One pop of the distant fireworks: time (s), sound variant, gain, pitch, yaw / pitch offset (deg).</summary>
        public struct Pop { public float time, gain, pitch, angle, tilt; public int variant; }
        public const int FireworkVariants = 12; // many variants: no snap pattern repeats audibly

        /// <summary>The pops' rhythm: 3-5 close together in the first ~0.3 s, the rest thinning out to ~2.3 s.</summary>
        public static List<Pop> FireworkSchedule(int seed)
        {
            var rng = new System.Random(seed);
            int count = 10 + rng.Next(7), dense = 3 + rng.Next(3);
            var list = new List<Pop>();
            for (int i = 0; i < count; i++)
            {
                float t;
                if (i < dense) t = 0.02f + 0.3f * i / dense + (float)rng.NextDouble() * 0.05f;
                else
                {
                    float u = (i - dense + 1) / (float)(count - dense);
                    t = 0.35f + 1.95f * Mathf.Pow(u, 1.5f) + ((float)rng.NextDouble() - 0.5f) * 0.12f;
                }
                list.Add(new Pop
                {
                    time = Mathf.Max(0f, t),
                    variant = rng.Next(FireworkVariants),
                    gain = Mathf.Lerp(0.45f, 1f, (float)rng.NextDouble()) * (1f - 0.35f * Mathf.Clamp01(t / 2.3f)),
                    pitch = Mathf.Lerp(0.85f, 1.2f, (float)rng.NextDouble()),
                    angle = ((float)rng.NextDouble() * 2f - 1f) * 7f,
                    tilt = ((float)rng.NextDouble() * 2f - 1f) * 4f,
                });
            }
            return list;
        }

        void PlayCascade(Vector3 at, Vector3 right, int count, float duration, float delay, int seed, float gain)
        {
            if (bars == null) return;
            foreach (var b in Cascade(count, duration, seed + Time.frameCount))
                Play(bars[b.note], at + right * (b.pan * barSpread), textVolume * gain * b.gain, b.detune, delay + b.time, 1.5f);
        }

        /// <summary>One brushed bar in a cascade: start time (s), bar (pentatonic index), gain, pan (-1..1), pitch.</summary>
        public struct Brush { public float time; public int note; public float gain, pan, detune; }

        /// <summary>
        /// A brush across the bars: <paramref name="count"/> bars over <paramref name="duration"/> s, a little uneven in
        /// time, wandering up and down the pentatonic scale without repeating a bar, each a little louder or softer.
        /// </summary>
        public static List<Brush> Cascade(int count, float duration, int seed)
        {
            var rng = new System.Random(seed);
            var list = new List<Brush>();
            int note = 1 + rng.Next(BarHz.Length - 3), dir = rng.Next(2) == 0 ? 1 : -1;
            for (int i = 0; i < count; i++)
            {
                float u = count > 1 ? i / (float)(count - 1) : 0f;
                float t = 0.04f + duration * Mathf.Pow(u, 0.9f) * 0.88f + ((float)rng.NextDouble() - 0.5f) * 0.08f;
                list.Add(new Brush
                {
                    time = Mathf.Max(0f, t),
                    note = note,
                    gain = Mathf.Lerp(0.55f, 1f, (float)rng.NextDouble()) * (1f - 0.25f * u),
                    pan = ((float)rng.NextDouble() * 2f - 1f),
                    detune = 1f + ((float)rng.NextDouble() - 0.5f) * 0.008f,
                });
                // next bar: one or two steps on, turning at the ends (and sometimes on its own), never the same bar
                int step = rng.NextDouble() < 0.7 ? 1 : 2;
                if (rng.NextDouble() < 0.25) dir = -dir;
                int nn = note + dir * step;
                if (nn < 0 || nn >= BarHz.Length) { dir = -dir; nn = note + dir * step; }
                note = Mathf.Clamp(nn, 0, BarHz.Length - 1);
            }
            return list;
        }

        /// <summary>Stops text chimes still playing or waiting (intro skipped or restarted).</summary>
        public void StopTextSounds()
        {
            foreach (var a in pool)
                if (a.clip == synIntro || a.clip == synInstruction || a.clip == introLineClip || a.clip == instructionClip ||
                    a.clip == synPaperOpen || a.clip == synPaperClose || a.clip == synSwellTone ||
                    (bars != null && Array.IndexOf(bars, a.clip) >= 0)) a.Stop();
        }

        public void PlayEndingText(Vector3 at)
        {
            if (!soundsEnabled || !textSounds || !endingTextSound) return;
            Play(endingClip != null ? endingClip : synEnding, at, textVolume * 1.1f, 1f, 0f, 1.5f);
        }

        public void PlayFlight(Vector3 towardWorld) => Directional(flightClip != null ? flightClip : synFlight, towardWorld, 0.8f);
        public void PlayArrival(Vector3 towardWorld) => Directional(arrivalClip != null ? arrivalClip : synArrival, towardWorld, 1f);
        public void PlayWishes(Vector3 towardWorld) => Directional(wishClip != null ? wishClip : synWish, towardWorld, 0.6f);

        // ------------------------------------------------------------------ progress

        void OnProgress(float v)
        {
            bool up = v > lastValue + 1e-4f;
            int stepBefore = Mathf.FloorToInt(lastValue * 4f + 1e-4f), stepNow = Mathf.FloorToInt(v * 4f + 1e-4f);
            lastValue = v;
            if (!up || stepNow > stepBefore) return; // a milestone plays its own sound
            if (!soundsEnabled || !progressSounds || !strikeSound) return;
            // shimmer from the Moon, a pentatonic step higher as the Moon fills
            int i = Mathf.Clamp(Mathf.RoundToInt(v * (Penta.Length - 1)), 0, Penta.Length - 1);
            Directional(strikeClip != null ? strikeClip : synStrike, MoonPosition(), 0.9f, strikeClip != null ? 1f : Penta[i] * 0.5f + 0.5f);
        }

        void OnMilestone(int m)
        {
            if (!soundsEnabled || !progressSounds) return;
            if (m >= 4) Directional(fullMoonClip != null ? fullMoonClip : synFull, MoonPosition(), 1.1f);
            else Directional(milestoneClip != null ? milestoneClip : synMilestone, MoonPosition(), 1f,
                             milestoneClip != null ? 1f : Penta[Mathf.Clamp(m - 1, 0, 2)]);
        }

        Vector3 MoonPosition()
        {
            var moon = GameObject.Find("MR_Moon");
            return moon != null ? moon.transform.position : transform.position + new Vector3(0f, 60f, 130f);
        }

        // ------------------------------------------------------------------ playback

        void Directional(AudioClip clip, Vector3 towardWorld, float gain, float pitch = 1f)
        {
            if (!soundsEnabled || !progressSounds) return;
            var cam = MoonStory.MainCamera();
            var head = cam != null ? cam.transform.position : transform.position + Vector3.up * 1.6f;
            var dir = towardWorld - head;
            var at = head + (dir.sqrMagnitude > 1e-4f ? dir.normalized : Vector3.forward) * directionalDistance;
            Play(clip, at, progressSfxVolume * gain, pitch, 0f, directionalDistance);
        }

        void Play(AudioClip clip, Vector3 at, float volume, float pitch, float delay, float minDistance)
        {
            if (clip == null || pool.Count == 0 || volume <= 0f) return;
            var a = pool[next];
            next = (next + 1) % pool.Count;
            a.Stop();
            a.transform.position = at;
            a.clip = clip;
            a.volume = volume;
            a.pitch = pitch;
            a.minDistance = minDistance; // full level at the source's own distance: direction, not distance
            if (delay > 0f) a.PlayDelayed(delay); else a.Play();
        }

        // ------------------------------------------------------------------ synthesis

        const int Rate = 48000;

        // hanging chime bars: a major pentatonic from G6 up (fundamentals ~1.6-3.5 kHz)
        public static readonly float[] BarHz = { 1567.98f, 1760f, 1975.53f, 2349.32f, 2637.02f, 3135.96f, 3520f };
        // free-free bar / tubular chime partials (inharmonic) and their levels
        static readonly float[] BarRatio = { 1f, 2.76f, 5.40f, 8.93f };
        static readonly float[] BarAmp = { 1f, 0.42f, 0.2f, 0.09f };

        /// <summary>
        /// One brushed bar: tubular partials, a 10 ms raised-cosine attack (no click), long decay (1.5-3 s, higher bars
        /// shorter), a slightly detuned twin of the fundamental for the slow shimmer, then the faint early-echo tail.
        /// </summary>
        public static float[] BarData(int index)
        {
            float f0 = BarHz[index];
            float decay = Mathf.Lerp(2.8f, 1.6f, index / (float)(BarHz.Length - 1));
            int n = (int)((decay * 1.25f + 0.3f) * Rate);
            var d = new float[n];
            const float attack = 0.010f;
            for (int k = 0; k < BarRatio.Length; k++)
            {
                float f = f0 * BarRatio[k];
                if (f > 17000f) continue;
                float tau = decay / (1f + 1.3f * k), a = BarAmp[k], ph = k * 0.9f + index;
                for (int i = 0; i < n; i++)
                {
                    float t = i / (float)Rate;
                    float env = (t < attack ? 0.5f - 0.5f * Mathf.Cos(Mathf.PI * t / attack) : 1f) * Mathf.Exp(-t / tau);
                    float v = Mathf.Sin(2f * Mathf.PI * f * t + ph);
                    if (k == 0) v += 0.35f * Mathf.Sin(2f * Mathf.PI * f * 1.0021f * t + 1.7f); // slow beating shimmer
                    d[i] += a * env * v;
                }
            }
            return Tail(d, 0.5f);
        }

        static AudioClip Clip(string name, float[] d)
        {
            var c = AudioClip.Create(name, d.Length, 1, Rate, false);
            c.SetData(d, 0);
            return c;
        }

        // ---- hanji paper, swell tone, distant fireworks

        /// <summary>Second-order band-pass (RBJ), applied in place.</summary>
        static void BandPass(float[] x, float f, float q)
        {
            float w = 2f * Mathf.PI * f / Rate, al = Mathf.Sin(w) / (2f * q), c = Mathf.Cos(w);
            float a0 = 1f + al, b0 = al / a0, b2 = -al / a0, a1 = -2f * c / a0, a2 = (1f - al) / a0;
            float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
            for (int i = 0; i < x.Length; i++)
            {
                float y = b0 * x[i] + b2 * x2 - a1 * y1 - a2 * y2;
                x2 = x1; x1 = x[i]; y2 = y1; y1 = y; x[i] = y;
            }
        }

        /// <summary>Second-order low-pass (RBJ), applied in place.</summary>
        static void LowPass(float[] x, float f, float q = 0.707f)
        {
            float w = 2f * Mathf.PI * f / Rate, al = Mathf.Sin(w) / (2f * q), c = Mathf.Cos(w);
            float a0 = 1f + al, b0 = (1f - c) / 2f / a0, b1 = (1f - c) / a0, a1 = -2f * c / a0, a2 = (1f - al) / a0;
            float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
            for (int i = 0; i < x.Length; i++)
            {
                float y = b0 * x[i] + b1 * x1 + b0 * x2 - a1 * y1 - a2 * y2;
                x2 = x1; x1 = x[i]; y2 = y1; y1 = y; x[i] = y;
            }
        }

        static float[] Noise(int n, System.Random rng)
        {
            var d = new float[n];
            for (int i = 0; i < n; i++) d[i] = (float)rng.NextDouble() * 2f - 1f;
            return d;
        }

        static void Normalize(float[] d, float peak)
        {
            float m = 0f;
            foreach (var v in d) m = Mathf.Max(m, Mathf.Abs(v));
            if (m > 1e-6f) for (int i = 0; i < d.Length; i++) d[i] *= peak / m;
            int f = Mathf.Min(d.Length, Rate / 50);
            for (int i = 0; i < f; i++) { d[i] *= i / (float)f; d[d.Length - 1 - i] *= i / (float)f; }
        }

        /// <summary>
        /// Hanji paper: soft friction "srrr" (band-passed noise whose grain wobbles like a sheet sliding off a roll) with
        /// small crinkle grains; <paramref name="close"/> adds a soft tap at <see cref="PaperCloseTap"/> as the roll shuts.
        /// </summary>
        public static float[] PaperData(float length, bool close, int seed)
        {
            var rng = new System.Random(seed);
            int n = (int)((length + (close ? 0.35f : 0.25f)) * Rate);
            var fr = Noise(n, rng);
            BandPass(fr, close ? 2100f : 2600f, 0.6f);
            var lowRub = Noise(n, rng);
            BandPass(lowRub, 700f, 0.8f);
            // friction envelope: swell in, carry, ease out; a slow random wobble for the paper's grain
            float wob = 0f, wobT = 0f;
            for (int i = 0; i < n; i++)
            {
                float t = i / (float)Rate, u = Mathf.Clamp01(t / length);
                float env = Mathf.SmoothStep(0f, 1f, Mathf.Clamp01(t / 0.08f)) * (1f - Mathf.SmoothStep(0.7f, 1.05f, u));
                if (i % 480 == 0) wobT = 0.55f + 0.45f * (float)rng.NextDouble();
                wob += (wobT - wob) * 0.004f;
                fr[i] = (fr[i] * 0.8f + lowRub[i] * 0.35f) * env * wob;
            }
            // crinkle grains: tiny bright crackles, denser in the middle of the movement
            int grains = (int)(length * 46f);
            for (int g = 0; g < grains; g++)
            {
                float t0 = length * Mathf.Pow((float)rng.NextDouble(), 0.9f) * 0.95f;
                int s0 = (int)(t0 * Rate), len = (int)((0.0015f + 0.004f * (float)rng.NextDouble()) * Rate);
                float a = (0.25f + 0.75f * (float)rng.NextDouble()) * 0.22f; // grains sit inside the rustle, not on top
                float f = 3000f + 4000f * (float)rng.NextDouble();
                for (int i = 0; i < len && s0 + i < n; i++)
                {
                    float w = 0.5f - 0.5f * Mathf.Cos(2f * Mathf.PI * i / len); // Hann: no hard click
                    fr[s0 + i] += a * w * ((float)rng.NextDouble() * 2f - 1f) * 0.6f + a * w * 0.4f * Mathf.Sin(2f * Mathf.PI * f * i / Rate);
                }
            }
            if (close)
            {
                // the roll closing: a soft low "tuk" (a damped thump and a little paper slap)
                int s0 = (int)(PaperCloseTap * Rate);
                var slap = Noise((int)(0.05f * Rate), rng);
                LowPass(slap, 1400f);
                for (int i = 0; s0 + i < n && i < (int)(0.25f * Rate); i++)
                {
                    float t = i / (float)Rate;
                    float env = (1f - Mathf.Exp(-t / 0.002f)) * Mathf.Exp(-t / 0.045f);
                    float thump = Mathf.Sin(2f * Mathf.PI * (150f - 40f * Mathf.Clamp01(t / 0.08f)) * t) * env * 1.4f;
                    float sl = i < slap.Length ? slap[i] * Mathf.Exp(-t / 0.012f) * 2.2f : 0f;
                    fr[s0 + i] += thump + sl;
                }
            }
            for (int i = 0; i < n; i++) fr[i] = (float)Math.Tanh(fr[i] * 2.5f); // gentle limiter: body up, spikes down
            Normalize(fr, 0.55f);
            return fr;
        }

        /// <summary>Time of the closing tap in the roll-up sound (s).</summary>
        public const float PaperCloseTap = 0.86f;
        /// <summary>Time of the loudest point of the swell tone (s).</summary>
        public const float SwellPeak = 0.95f;

        /// <summary>
        /// "Jiiiing~": a rubbed singing bowl / wet glass rim. A clear tone (few partials) that blooms up to
        /// <see cref="SwellPeak"/> and fades, rising a touch in pitch, with the bowl's slow beating.
        /// </summary>
        public static float[] SwellToneData()
        {
            float len = 3.4f, f0 = 1174.66f; // D6, in the chime's pentatonic
            int n = (int)(len * Rate);
            var d = new float[n];
            double ph1 = 0, ph2 = 0, ph3 = 0, ph4 = 0;
            for (int i = 0; i < n; i++)
            {
                float t = i / (float)Rate;
                // bloom: slow rise to the peak, then a long soft fade
                float rise = Mathf.SmoothStep(0f, 1f, Mathf.Clamp01(t / SwellPeak));
                float env = rise * Mathf.Exp(-Mathf.Max(0f, t - SwellPeak) / 0.75f);
                float glide = 1f + 0.012f * Mathf.SmoothStep(0f, 1f, Mathf.Clamp01(t / 1.6f)); // up a touch
                float f = f0 * glide;
                ph1 += 2 * Math.PI * f / Rate;
                ph2 += 2 * Math.PI * (f + 1.6f) / Rate;          // slow beating like a bowl
                ph3 += 2 * Math.PI * f * 2.0 / Rate;
                ph4 += 2 * Math.PI * f * 2.92 / Rate;            // faint bowl overtone
                float rub = 1f + 0.04f * Mathf.Sin(2f * Mathf.PI * 5.2f * t); // the finger going round
                d[i] = env * rub * ((float)Math.Sin(ph1) * 0.62f + (float)Math.Sin(ph2) * 0.38f
                                    + (float)Math.Sin(ph3) * 0.1f * rise + (float)Math.Sin(ph4) * 0.035f);
            }
            var o = Tail(d, 0.5f);
            return o;
        }

        /// <summary>
        /// One small distant firework crackle, built to sound natural rather than synthetic: 3-8 snaps at irregular
        /// (Poisson) intervals, each with its own random band (800 Hz-8 kHz, random Q), length (2-40 ms), decay and a
        /// fine flutter; a noise "sizzle" after them (sparse grains, no sine tones); a faint low air "whum" and a wide,
        /// soft far-away tail. Nothing repeats a fixed frequency.
        /// </summary>
        public static float[] FireworkPopData(int seed)
        {
            var rng = new System.Random(seed * 7919 + 13);
            float R() => (float)rng.NextDouble();
            float len = 1.4f;
            int n = (int)(len * Rate);
            var d = new float[n];
            // snaps: Poisson-like gaps (exponential, mean ~18 ms), the first one the strongest
            int snaps = 3 + rng.Next(6);
            float t0 = 0f;
            for (int c = 0; c < snaps; c++)
            {
                if (c > 0) t0 += -Mathf.Log(Mathf.Max(1e-4f, R())) * 0.018f;
                float dur = Mathf.Exp(Mathf.Lerp(Mathf.Log(0.002f), Mathf.Log(0.04f), R())); // 2-40 ms, log-uniform
                float fc = Mathf.Exp(Mathf.Lerp(Mathf.Log(800f), Mathf.Log(8000f), R()));    // 0.8-8 kHz
                int s0 = (int)(t0 * Rate), m = (int)(dur * 2.5f * Rate) + 8;
                var nz = Noise(m, rng);
                BandPass(nz, fc, Mathf.Lerp(0.5f, 2f, R()));
                if (R() < 0.5f) BandPass(nz, fc * Mathf.Lerp(0.7f, 1.4f, R()), 0.7f); // some snaps a little purer
                float g = (c == 0 ? 1f : Mathf.Lerp(0.3f, 0.95f, R())) * Mathf.Exp(-t0 / 0.25f);
                float att = Mathf.Lerp(0.0002f, 0.0012f, R()), tau = dur * Mathf.Lerp(0.25f, 0.6f, R());
                float fl = 0f;
                for (int i = 0; i < m && s0 + i < n; i++)
                {
                    float t = i / (float)Rate;
                    fl += ((R() * 2f - 1f) - fl) * 0.08f;                      // fine, irregular flutter
                    float env = (1f - Mathf.Exp(-t / att)) * Mathf.Exp(-t / tau) * (1f + 0.45f * fl);
                    d[s0 + i] += g * env * nz[i] * 2.2f;
                }
            }
            // sizzle: sparse tiny noise grains thinning out over 60-250 ms (no tones)
            float sz = Mathf.Lerp(0.06f, 0.25f, R());
            var hiss = Noise(n, rng);
            BandPass(hiss, Mathf.Lerp(2500f, 6500f, R()), 0.6f);
            float grain = 0f;
            for (int i = 0; i < (int)((t0 + sz * 3f) * Rate) && i < n; i++)
            {
                float t = i / (float)Rate;
                if (R() < 0.004f) grain = Mathf.Lerp(0.3f, 1f, R()); // a new grain now and then
                grain *= 0.996f;
                d[i] += hiss[i] * grain * 0.18f * Mathf.Exp(-t / sz);
            }
            // a faint low air "whum" (distance), very small
            var low = Noise(n, rng);
            LowPass(low, 140f);
            for (int i = 0; i < n; i++)
            {
                float t = i / (float)Rate;
                d[i] += low[i] * 0.45f * (1f - Mathf.Exp(-t / 0.01f)) * Mathf.Exp(-t / 0.18f);
            }
            // wide, soft far-away tail: combs at irregular delays + one all-pass to smear the echoes
            var wet = new float[n];
            int[] dl = { (int)(0.0311f * Rate), (int)(0.0419f * Rate), (int)(0.0533f * Rate), (int)(0.0677f * Rate), (int)(0.0791f * Rate) };
            foreach (var L in dl)
            {
                var c = new float[n];
                for (int i = 0; i < n; i++) c[i] = d[i] + (i >= L ? c[i - L] * 0.72f : 0f);
                for (int i = 0; i < n; i++) wet[i] += c[i] * 0.2f;
            }
            int ap = (int)(0.0051f * Rate); var apb = new float[n];
            for (int i = 0; i < n; i++) { float x = wet[i] + (i >= ap ? apb[i - ap] * 0.6f : 0f); apb[i] = x; wet[i] = (i >= ap ? apb[i - ap] : 0f) - 0.6f * x; }
            LowPass(wet, 6000f); // air softens the tail a little (the snaps stay bright)
            for (int i = 0; i < n; i++) d[i] = d[i] * 0.8f + wet[i] * 0.4f;
            // gentle limiter so the short snaps carry about the same loudness as before at the same volume
            float mx = 0f; foreach (var v in d) mx = Mathf.Max(mx, Mathf.Abs(v));
            if (mx > 1e-6f) for (int i = 0; i < n; i++) d[i] = (float)Math.Tanh(d[i] / mx * 1.6f);
            Normalize(d, 0.6f);
            return d;
        }

        /// <summary>The whole distant firework (all pops of <see cref="FireworkSchedule"/>), stereo, for the preview.</summary>
        public static void RenderFireworkWav(string path, float volume, int seed)
        {
            float lead = 0.3f, len = 5f;
            int n = (int)(len * Rate);
            var l = new float[n]; var r = new float[n];
            var pops = new float[FireworkVariants][];
            for (int v = 0; v < pops.Length; v++) pops[v] = FireworkPopData(v);
            foreach (var p in FireworkSchedule(seed))
            {
                var src = pops[p.variant];
                float pan = Mathf.Clamp(p.angle / 7f, -1f, 1f) * 0.35f;
                float g = volume * p.gain * 0.71f, gl = g * Mathf.Sqrt(1f - pan), gr = g * Mathf.Sqrt(1f + pan);
                int s0 = (int)((lead + p.time) * Rate);
                for (int i = 0; ; i++)
                {
                    float si = i * p.pitch; int j = (int)si; if (j + 1 >= src.Length) break;
                    int o = s0 + i; if (o >= n) break;
                    float v = Mathf.Lerp(src[j], src[j + 1], si - j);
                    l[o] += v * gl; r[o] += v * gr;
                }
            }
            WriteWav(path, l, r);
        }

        void Synthesize()
        {
            synPaperOpen = Clip("MR_PaperUnroll", PaperData(0.7f, false, 21));
            synPaperClose = Clip("MR_PaperRollUp", PaperData(0.9f, true, 37));
            synSwellTone = Clip("MR_SwellTone", SwellToneData());
            fireworkPops = new AudioClip[FireworkVariants];
            for (int v = 0; v < FireworkVariants; v++) fireworkPops[v] = Clip("MR_FireworkPop_" + v, FireworkPopData(v));
            bars = new AudioClip[BarHz.Length];
            for (int i = 0; i < bars.Length; i++)
            {
                var d = BarData(i);
                bars[i] = AudioClip.Create("MR_ChimeBar_" + i, d.Length, 1, Rate, false);
                bars[i].SetData(d, 0);
            }
            // soft wind chime: a few bell-like partials, fast attack, long decay, faint early echoes as a reverb tail
            synIntro = Bell("MR_IntroChime", 1.9f, 1.6f, (659.25f, 1f), (659.25f * 2.01f, 0.35f), (659.25f * 2.76f, 0.18f));
            synInstruction = Bell("MR_InstructionChime", 2.2f, 1.8f, (783.99f, 0.9f), (987.77f, 0.7f), (1174.66f, 0.6f), (1567.98f, 0.35f), (2349.3f, 0.12f));
            synEnding = Bell("MR_EndingChime", 2.6f, 2.0f, (523.25f, 0.8f), (659.25f, 0.7f), (783.99f, 0.7f), (1046.5f, 0.45f), (2093f, 0.12f));
            synStrike = Bell("MR_MoonShimmer", 1.0f, 0.8f, (1318.5f, 0.8f), (1975.5f, 0.45f), (2637f, 0.25f));
            synMilestone = Bell("MR_MilestoneChord", 2.0f, 1.5f, (1046.5f, 0.8f), (1318.5f, 0.7f), (1568f, 0.6f), (2093f, 0.35f));
            synFull = Swell("MR_FullMoonSwell", 2.6f, 0.9f, 1.4f, (523.25f, 0.7f), (659.25f, 0.6f), (783.99f, 0.6f), (1046.5f, 0.5f), (1568f, 0.25f), (2093f, 0.15f));
            synFlight = Whoosh("MR_FlightWhoosh", 3.4f);
            synArrival = Sparkles("MR_ArrivalBurst", 2.2f, 28, 0.05f, 1.1f, 1400f, 3200f, 0.35f);
            synWish = Sparkles("MR_WishTwinkles", 3.5f, 18, 0.1f, 3.2f, 1600f, 2800f, 0.5f);

            foreach (var c in new[] { synIntro, synInstruction, synEnding, synStrike, synMilestone, synFull, synFlight, synArrival, synWish })
                Debug.Log($"[Moon Rabbit] Synth clip {c.name}: {c.length:F2} s, peak {Peak(c):F2}");
        }

        static float Peak(AudioClip c)
        {
            var d = new float[c.samples * c.channels];
            c.GetData(d, 0);
            float p = 0f;
            foreach (var x in d) p = Mathf.Max(p, Mathf.Abs(x));
            return p;
        }

        static AudioClip Make(string name, float[] d, float peak = 0.7f)
        {
            var o = Tail(d, peak);
            var clip = AudioClip.Create(name, o.Length, 1, Rate, false);
            clip.SetData(o, 0);
            return clip;
        }

        /// <summary>Faint reverb-like tail (a few early echoes), normalized to a gentle peak, last 50 ms faded.</summary>
        static float[] Tail(float[] d, float peak)
        {
            var o = (float[])d.Clone();
            int[] taps = { (int)(0.037f * Rate), (int)(0.053f * Rate), (int)(0.079f * Rate), (int)(0.113f * Rate), (int)(0.161f * Rate) };
            float[] g = { 0.22f, 0.18f, 0.14f, 0.11f, 0.08f };
            for (int t = 0; t < taps.Length; t++)
                for (int i = taps[t]; i < o.Length; i++) o[i] += d[i - taps[t]] * g[t];
            float m = 0f;
            foreach (var x in o) m = Mathf.Max(m, Mathf.Abs(x));
            if (m > 1e-6f) for (int i = 0; i < o.Length; i++) o[i] *= peak / m;
            // fade the last 50 ms so nothing clicks
            int f = Mathf.Min(o.Length, Rate / 20);
            for (int i = 0; i < f; i++) o[o.Length - 1 - i] *= i / (float)f;
            return o;
        }

        // ------------------------------------------------------------------ preview (Editor / tests)

        /// <summary>Writes one synthesized sound (mono → centred stereo) at <paramref name="gain"/> to a 16-bit WAV.</summary>
        public static void RenderSoundWav(string path, float[] mono, float gain, float lead = 0.1f)
        {
            int pad = (int)(lead * Rate), n = mono.Length + pad + Rate / 2;
            var l = new float[n];
            for (int i = 0; i < mono.Length; i++) l[pad + i] = mono[i] * gain * 0.71f;
            WriteWav(path, l, l);
        }

        static void WriteWav(string path, float[] l, float[] r)
        {
            int n = l.Length;
            using (var fs = new System.IO.FileStream(path, System.IO.FileMode.Create))
            using (var w = new System.IO.BinaryWriter(fs))
            {
                int bytes = n * 4;
                w.Write(System.Text.Encoding.ASCII.GetBytes("RIFF")); w.Write(36 + bytes);
                w.Write(System.Text.Encoding.ASCII.GetBytes("WAVEfmt ")); w.Write(16); w.Write((short)1); w.Write((short)2);
                w.Write(Rate); w.Write(Rate * 4); w.Write((short)4); w.Write((short)16);
                w.Write(System.Text.Encoding.ASCII.GetBytes("data")); w.Write(bytes);
                for (int i = 0; i < n; i++)
                {
                    w.Write((short)Mathf.Clamp(Mathf.RoundToInt(l[i] * 32767f), -32768, 32767));
                    w.Write((short)Mathf.Clamp(Mathf.RoundToInt(r[i] * 32767f), -32768, 32767));
                }
            }
        }

        /// <summary>
        /// Renders brushes to a stereo 16-bit WAV at the in-game level (textVolume × gain, ~-2 dB for the text's
        /// distance), so the sound can be checked without a headset. Each item: (start s, count, duration s, seed, gain).
        /// </summary>
        public static void RenderPreviewWav(string path, float length, float textVolume, params (float start, int count, float duration, int seed, float gain)[] brushes)
        {
            int n = (int)(length * Rate);
            var l = new float[n]; var r = new float[n];
            var data = new float[BarHz.Length][];
            for (int i = 0; i < data.Length; i++) data[i] = BarData(i);
            foreach (var br in brushes)
                foreach (var b in Cascade(br.count, br.duration, br.seed))
                {
                    var src = data[b.note];
                    float g = textVolume * br.gain * b.gain * 0.8f;
                    float gl = g * Mathf.Sqrt(0.5f * (1f - b.pan * 0.5f)), gr = g * Mathf.Sqrt(0.5f * (1f + b.pan * 0.5f));
                    int s0 = (int)((br.start + b.time) * Rate);
                    for (int i = 0; i < src.Length; i++)
                    {
                        // pitch (detune) by linear resampling
                        float si = i * b.detune; int j = (int)si; if (j + 1 >= src.Length) break;
                        float v = Mathf.Lerp(src[j], src[j + 1], si - j);
                        int o = s0 + i; if (o >= n) break;
                        l[o] += v * gl; r[o] += v * gr;
                    }
                }
            using (var fs = new System.IO.FileStream(path, System.IO.FileMode.Create))
            using (var w = new System.IO.BinaryWriter(fs))
            {
                int bytes = n * 4;
                w.Write(System.Text.Encoding.ASCII.GetBytes("RIFF")); w.Write(36 + bytes);
                w.Write(System.Text.Encoding.ASCII.GetBytes("WAVEfmt ")); w.Write(16); w.Write((short)1); w.Write((short)2);
                w.Write(Rate); w.Write(Rate * 4); w.Write((short)4); w.Write((short)16);
                w.Write(System.Text.Encoding.ASCII.GetBytes("data")); w.Write(bytes);
                for (int i = 0; i < n; i++)
                {
                    w.Write((short)Mathf.Clamp(Mathf.RoundToInt(l[i] * 32767f), -32768, 32767));
                    w.Write((short)Mathf.Clamp(Mathf.RoundToInt(r[i] * 32767f), -32768, 32767));
                }
            }
        }

        static AudioClip Bell(string name, float length, float decay, params (float f, float a)[] partials)
        {
            int n = (int)(length * Rate);
            var d = new float[n];
            for (int k = 0; k < partials.Length; k++)
            {
                var (f, a) = partials[k];
                float tau = decay / (1f + 0.5f * k);       // higher partials fade sooner
                float ph = k * 1.3f;
                for (int i = 0; i < n; i++)
                {
                    float t = i / (float)Rate;
                    float env = (1f - Mathf.Exp(-t / 0.006f)) * Mathf.Exp(-t / tau * 2.2f);
                    d[i] += a * env * Mathf.Sin(2f * Mathf.PI * f * t + ph);
                }
            }
            return Make(name, d);
        }

        static AudioClip Swell(string name, float length, float rise, float decay, params (float f, float a)[] partials)
        {
            int n = (int)(length * Rate);
            var d = new float[n];
            for (int k = 0; k < partials.Length; k++)
            {
                var (f, a) = partials[k];
                float delay = k * 0.06f; // the chord blooms upward
                for (int i = 0; i < n; i++)
                {
                    float t = i / (float)Rate - delay;
                    if (t < 0f) continue;
                    float up = Mathf.SmoothStep(0f, 1f, Mathf.Clamp01(t / rise));
                    float env = up * Mathf.Exp(-Mathf.Max(0f, t - rise) / decay * 1.5f);
                    float vib = 1f + 0.002f * Mathf.Sin(2f * Mathf.PI * 5f * t);
                    d[i] += a * env * Mathf.Sin(2f * Mathf.PI * f * vib * t);
                }
            }
            return Make(name, d, 0.65f);
        }

        static AudioClip Whoosh(string name, float length)
        {
            int n = (int)(length * Rate);
            var d = new float[n];
            var rng = new System.Random(3);
            float lp = 0f;
            for (int i = 0; i < n; i++)
            {
                float t = i / (float)n;
                float env = Mathf.Sin(Mathf.PI * Mathf.Pow(t, 0.7f)) * (1f - t * 0.3f);
                float cutoff = Mathf.Lerp(0.02f, 0.12f, Mathf.Sin(Mathf.PI * t)); // the swish opens and closes
                float noise = (float)rng.NextDouble() * 2f - 1f;
                lp += (noise - lp) * cutoff;
                d[i] = lp * env * 2.5f;
            }
            // sparkle trail on top
            var s = SparkleData(length, 14, 0.25f, length * 0.95f, 1800f, 3000f, 0.3f, 11);
            for (int i = 0; i < n; i++) d[i] += s[i] * 0.35f;
            return Make(name, d, 0.5f);
        }

        static AudioClip Sparkles(string name, float length, int count, float from, float to, float fLo, float fHi, float pingDecay)
            => Make(name, SparkleData(length, count, from, to, fLo, fHi, pingDecay, name.GetHashCode()), 0.6f);

        static float[] SparkleData(float length, int count, float from, float to, float fLo, float fHi, float pingDecay, int seed)
        {
            int n = (int)(length * Rate);
            var d = new float[n];
            var rng = new System.Random(seed);
            for (int p = 0; p < count; p++)
            {
                float t0 = Mathf.Lerp(from, to, (float)Math.Pow(rng.NextDouble(), 0.8));
                float f = Mathf.Lerp(fLo, fHi, (float)rng.NextDouble());
                float a = Mathf.Lerp(0.3f, 1f, (float)rng.NextDouble()) * (1f - 0.5f * (t0 - from) / Mathf.Max(0.01f, to - from));
                int s0 = (int)(t0 * Rate), len = Mathf.Min(n - s0, (int)(pingDecay * 3f * Rate));
                for (int i = 0; i < len; i++)
                {
                    float t = i / (float)Rate;
                    float env = (1f - Mathf.Exp(-t / 0.003f)) * Mathf.Exp(-t / pingDecay * 2f);
                    d[s0 + i] += a * env * (Mathf.Sin(2f * Mathf.PI * f * t) + 0.3f * Mathf.Sin(2f * Mathf.PI * f * 2.7f * t));
                }
            }
            return d;
        }
    }
}
