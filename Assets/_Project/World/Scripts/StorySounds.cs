using System;
using System.Collections.Generic;
using UnityEngine;

namespace MoonRabbit.World
{
    /// <summary>
    /// Story sound effects, synthesized in code (no audio assets): soft wind-chime tones when story text appears,
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
        public bool soundsEnabled = true;
        public bool textSounds = true;
        public bool progressSounds = true;
        [Tooltip("Shimmer on every progress step. Turn off when the rhythm game plays its own hit sounds.")]
        public bool strikeSound = true;
        [Range(0f, 1f)] public float textVolume = 0.35f;
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
        [Range(1, 12)] public int voices = 6;

        // pentatonic steps (major): each intro line and each progress step a little higher
        static readonly float[] Penta = { 1f, 9f / 8f, 5f / 4f, 3f / 2f, 5f / 3f, 2f, 9f / 4f, 5f / 2f, 3f };

        MoonProgress progress;
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
            try
            {
                var available = AudioSettings.GetSpatializerPluginNames();
                if (available == null) return false;
                foreach (var a in available) if (a == name) return true;
            }
            catch (Exception) { }
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
            try { avail = string.Join("|", AudioSettings.GetSpatializerPluginNames() ?? new string[0]); } catch (Exception) { }
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

        /// <summary>Chime for an intro line at the text's position (pitch by line), delayed if needed (instruction bounce).</summary>
        public void PlayIntroLine(Vector3 at, int lineIndex, bool instruction, float delay = 0f)
        {
            if (!soundsEnabled || !textSounds) return;
            var clip = instruction ? (instructionClip != null ? instructionClip : synInstruction)
                                   : (introLineClip != null ? introLineClip : synIntro);
            float pitch = instruction || introLineClip != null ? 1f : Penta[Mathf.Clamp(lineIndex, 0, 4)];
            Play(clip, at, textVolume * (instruction ? 1.15f : 1f), pitch, delay, 1.5f);
        }

        /// <summary>Stops text chimes still playing or waiting (intro skipped or restarted).</summary>
        public void StopTextSounds()
        {
            foreach (var a in pool)
                if (a.clip == synIntro || a.clip == synInstruction || a.clip == introLineClip || a.clip == instructionClip) a.Stop();
        }

        public void PlayEndingText(Vector3 at)
        {
            if (!soundsEnabled || !textSounds) return;
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

        void Synthesize()
        {
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
            // faint reverb-like tail: a few early echoes, then normalize to a gentle peak
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
            var clip = AudioClip.Create(name, o.Length, 1, Rate, false);
            clip.SetData(o, 0);
            return clip;
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
