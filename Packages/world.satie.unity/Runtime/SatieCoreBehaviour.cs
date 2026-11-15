// SatieCoreBehaviour.cs: plays a compiled SATIE scene (scene.satp + its manifest)
// through the portable core inside Unity. The core does the scheduling, levels
// and mixing; this component only loads the files, hands the audio callback to
// the core, and reports where things are. See README.md next to it.
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Satie.Core
{
    /// The calls a game makes. They do nothing until a SatieCoreBehaviour has loaded a scene.
    public static class SatieCore
    {
        static SatieCoreBehaviour B { get { return SatieCoreBehaviour.Instance; } }
        public static int Fire(string eventName) { return B != null ? B.Fire(eventName, null, null, false, Vector3.zero) : 0; }
        public static int Fire(string eventName, Vector3 position, string surface = null) { return B != null ? B.Fire(eventName, null, surface, true, position) : 0; }
        public static int HostEvent(string host, string eventName, string surface = null) { return B != null ? B.Fire(eventName, host, surface, false, Vector3.zero) : 0; }
        public static void SetHostSurface(string host, string surface) { if (B != null && B.Session != null) B.Session.SetHostSurface(host, surface); }
        public static bool Signal(string name, float value) { return B != null && B.Signal(name, value); }
        public static bool State(string name) { return B != null && B.State(name); }
        public static void Bind(string host, Transform target) { if (B != null) B.Bind(host, target); }
        public static void Unbind(string host) { if (B != null) B.Unbind(host); }
    }

    public enum SatieOutput
    {
        /// The core decides what sounds and how loud; every positioned voice plays through its own
        /// AudioSource, so Unity's panning or the project's spatializer plugin gives it its direction.
        EngineSpatializer,
        /// The core's own finished stereo mix on one AudioSource. No engine spatialization.
        StereoMix,
    }

    /// One voice slot's stream on its own AudioSource. The source plays a clip of ones, so whatever
    /// reaches this filter is the engine's panning gains (or plain ones when a spatializer plugin runs
    /// after the effects); multiplying by the voice puts the voice where the source is either way.
    public class SatieVoiceOutput : MonoBehaviour
    {
        internal SatieCoreBehaviour owner;
        internal int slot;
        long taken;
        float[] mono = new float[0];

        void OnAudioFilterRead(float[] data, int channels)
        {
            SatieCoreBehaviour o = owner;
            int frames = channels > 0 ? data.Length / channels : 0;
            if (mono.Length < frames) mono = new float[frames];
            if (o == null || !o.TakeVoice(slot, ref taken, mono, frames)) { Array.Clear(data, 0, data.Length); return; }
            for (int f = 0; f < frames; f++)
                for (int c = 0; c < channels; c++)
                    data[f * channels + c] *= mono[f];
        }
    }

    [RequireComponent(typeof(AudioSource))]
    public class SatieCoreBehaviour : MonoBehaviour
    {
        public static SatieCoreBehaviour Instance { get; private set; }

        [Tooltip("scene.satp renamed scene.satp.bytes, as a TextAsset. Empty = Resources/SatieCore/scene.satp")]
        public TextAsset bundle;
        [Tooltip("scene.manifest.json. Empty = Resources/SatieCore/scene.manifest")]
        public TextAsset manifestJson;
        [Tooltip("The ears. Empty = the scene's AudioListener, else the main camera.")]
        public Transform listener;
        [Range(0f, 2f)] public float masterGain = 1f;
        public int maxVoices = 64;
        [Tooltip("Who gives each sound its direction. Set before the scene loads.")]
        public SatieOutput output = SatieOutput.EngineSpatializer;

        public SatieSession Session { get; private set; }
        readonly Dictionary<string, Transform> bound = new Dictionary<string, Transform>();
        readonly HashSet<string> warned = new HashSet<string>();
        float[] block = new float[0];
        volatile bool live;
        // Object output. The audio thread renders one block for every stream; the main thread reads the
        // voice table it last published.
        readonly object blockLock = new object();
        Transform[] voiceNodes = new Transform[0];
        float[][] voiceBlocks = new float[0][];
        float[] ambientBlock = new float[0];
        SatieVoice[] tableWork = new SatieVoice[0], tablePublished = new SatieVoice[0];
        long renderedBlocks, ambientTaken;
        int blockFrames;
        float nextBindTry;
        int bindTries;
        // The mixer's side (SatieMixerWindow): what each source reached, the core's counters, and an
        // audition gain per source. None of it is part of the scene; a mixer writes its decisions
        // to the edit log and the script changes there.
        float[] levelWork = new float[0], levelHeld = new float[0];
        float[] trimDb = new float[0], sentGain = new float[0];
        bool[] muted = new bool[0], soloed = new bool[0];
        SatieStats statsPublished;
        readonly Dictionary<string, float> lastSignals = new Dictionary<string, float>();
        string lastState;
        bool stateSet;

        /// Raised after a scene is loaded or reloaded (the manifest may have changed).
        public event Action Loaded;
        /// How many times a scene has been loaded into this component.
        public int Loads { get; private set; }
        /// The core's counters as of the last rendered block.
        public SatieStats Stats { get { return statsPublished; } }

        // A game assigns nothing: when a compiled scene is in Resources/SatieCore and the scene has no
        // SatieCoreBehaviour of its own, one starts after the first scene loads and outlives scene changes.
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        static void Boot()
        {
            if (Instance != null) return;
            if (Resources.Load<TextAsset>("SatieCore/scene.manifest") == null) return;
            GameObject go = new GameObject("Satie");
            go.AddComponent<SatieCoreBehaviour>();
            DontDestroyOnLoad(go);
        }

        void Awake()
        {
            if (Instance != null && Instance != this) { Destroy(this); return; }
            Instance = this;
            TextAsset bytes = bundle != null ? bundle : Resources.Load<TextAsset>("SatieCore/scene.satp");
            TextAsset json = manifestJson != null ? manifestJson : Resources.Load<TextAsset>("SatieCore/scene.manifest");
            if (bytes == null || json == null) { Debug.LogError("Satie: no compiled scene (scene.satp.bytes and scene.manifest.json)"); return; }
            try
            {
                Session = new SatieSession(bytes.bytes, JsonUtility.FromJson<SatieManifest>(json.text), AudioSettings.outputSampleRate, maxVoices);
            }
            catch (Exception e) { Debug.LogError(e.Message); return; }
            foreach (string line in Session.Manifest.notRendered) Debug.Log("Satie, not rendered in this profile: " + line);
            SizeMixer(null);
            BindByName();
            if (output == SatieOutput.EngineSpatializer && Session.EnableObjects()) BuildVoiceSources();
            // OnAudioFilterRead runs only while the source plays: give it a silent loop to play.
            AudioSource source = GetComponent<AudioSource>();
            source.clip = AudioClip.Create("satie-core", AudioSettings.outputSampleRate, 2, AudioSettings.outputSampleRate, false);
            source.loop = true;
            source.spatialBlend = 0f;
            source.Play();
            live = true;
            Loads++;
            if (Loaded != null) Loaded();
        }

        /// Swap in a newly compiled scene while playing (the same pair of files Awake reads). Signals,
        /// the state and the mixer's audition settings carry over by name. False leaves the old scene playing.
        public bool Reload(byte[] bytes, string json)
        {
            if (Session == null || bytes == null || json == null) return false;
            SatieSession next;
            try { next = new SatieSession(bytes, JsonUtility.FromJson<SatieManifest>(json), AudioSettings.outputSampleRate, maxVoices); }
            catch (Exception e) { Debug.LogError("Satie: reload refused. " + e.Message); return false; }
            bool objects = voiceNodes.Length > 0;
            if (objects && (!next.EnableObjects() || next.VoiceSlots != voiceNodes.Length))
            {
                next.Dispose();
                Debug.LogError("Satie: reload refused, the new scene cannot use the voice sources already built.");
                return false;
            }
            SatieManifest before = Session.Manifest;
            // Every render path holds blockLock, so the old core is idle while it is swapped and freed.
            lock (blockLock)
            {
                SatieSession old = Session;
                Session = next;
                old.Dispose();
            }
            SizeMixer(before);
            bindTries = 0;
            warned.Clear();
            BindByName();
            foreach (KeyValuePair<string, float> pair in lastSignals) Session.Signal(pair.Key, pair.Value);
            if (stateSet) Session.State(lastState);
            Loads++;
            if (Loaded != null) Loaded();
            return true;
        }

        // The mixer's arrays, one entry per source; settings follow a source's id across a reload.
        void SizeMixer(SatieManifest before)
        {
            SatieSourceEntry[] sources = Session.Manifest.sources;
            float[] trims = new float[sources.Length];
            bool[] mutes = new bool[sources.Length], solos = new bool[sources.Length];
            float[] sent = new float[sources.Length];
            for (int i = 0; i < sources.Length; i++)
            {
                sent[i] = 1f;
                if (before == null) continue;
                for (int j = 0; j < before.sources.Length && j < trimDb.Length; j++)
                    if (before.sources[j].id == sources[i].id) { trims[i] = trimDb[j]; mutes[i] = muted[j]; solos[i] = soloed[j]; break; }
            }
            lock (blockLock)
            {
                trimDb = trims; muted = mutes; soloed = solos; sentGain = sent;
                levelWork = new float[sources.Length];
                levelHeld = new float[sources.Length];
            }
        }

        public bool Signal(string name, float value)
        {
            if (Session == null || !Session.Signal(name, value)) return false;
            lastSignals[name] = value;
            return true;
        }

        public bool State(string name)
        {
            if (Session == null || !Session.State(name)) return false;
            lastState = name;
            stateSet = true;
            return true;
        }
        /// The last value sent for a signal, or its rest value.
        public float SignalValue(string name)
        {
            float v;
            if (lastSignals.TryGetValue(name, out v)) return v;
            if (Session != null) foreach (SatieSignalEntry s in Session.Manifest.signalList) if (s.name == name) return s.rest;
            return 0f;
        }
        public string CurrentState { get { return stateSet ? lastState : null; } }

        // ── The mixer's audition: heard now, written nowhere ──
        public float TrimDb(int source) { return source >= 0 && source < trimDb.Length ? trimDb[source] : 0f; }
        public bool Muted(int source) { return source >= 0 && source < muted.Length && muted[source]; }
        public bool Soloed(int source) { return source >= 0 && source < soloed.Length && soloed[source]; }
        public void SetTrimDb(int source, float db) { if (source >= 0 && source < trimDb.Length) trimDb[source] = Math.Min(12f, Math.Max(-40f, db)); }
        public void SetMuted(int source, bool on) { if (source >= 0 && source < muted.Length) muted[source] = on; }
        public void SetSoloed(int source, bool on) { if (source >= 0 && source < soloed.Length) soloed[source] = on; }
        /// The loudest peak a source reached since the last call, linear. Reading it starts a new hold.
        public float TakeLevel(int source)
        {
            float[] held = levelHeld;
            if (source < 0 || source >= held.Length) return 0f;
            float v = held[source];
            held[source] = 0f;
            return v;
        }
        public Transform BoundTo(string host) { Transform t; return host != null && bound.TryGetValue(host, out t) ? t : null; }

        void SendMonitor()
        {
            if (Session == null || !Session.CanMonitor) return;
            bool anySolo = false;
            for (int i = 0; i < soloed.Length; i++) anySolo |= soloed[i];
            for (int i = 0; i < sentGain.Length; i++)
            {
                bool heard = anySolo ? soloed[i] : !muted[i];
                float gain = heard ? (float)Math.Pow(10.0, trimDb[i] / 20.0) : 0f;
                if (Math.Abs(gain - sentGain[i]) < 1e-6f) continue;
                if (Session.Monitor(i, gain)) sentGain[i] = gain;
            }
        }

        // Audio thread, inside blockLock, right after a render.
        void Publish(SatieSession session)
        {
            float[] work = levelWork, held = levelHeld;
            if (session.Levels(work))
                for (int i = 0; i < work.Length && i < held.Length; i++)
                    if (work[i] > held[i]) held[i] = work[i];
            statsPublished = session.Stats();
        }

        void OnDestroy()
        {
            live = false;
            AudioSource source = GetComponent<AudioSource>();
            if (source != null) source.Stop();
            if (Session != null) { Session.Dispose(); Session = null; }
            if (Instance == this) Instance = null;
        }

        void BuildVoiceSources()
        {
            int slots = Session.VoiceSlots, rate = AudioSettings.outputSampleRate;
            bool plugin = !string.IsNullOrEmpty(AudioSettings.GetSpatializerPluginName());
            // A clip of ones: the voice filter multiplies it by the voice (see SatieVoiceOutput).
            AudioClip ones = AudioClip.Create("satie-ones", rate, 1, rate, false);
            float[] fill = new float[rate];
            for (int i = 0; i < fill.Length; i++) fill[i] = 1f;
            ones.SetData(fill, 0);
            voiceNodes = new Transform[slots];
            voiceBlocks = new float[slots][];
            ambientBlock = new float[4096 * 2];
            tableWork = new SatieVoice[slots];
            tablePublished = new SatieVoice[slots];
            for (int slot = 0; slot < slots; slot++)
            {
                voiceBlocks[slot] = new float[4096];
                GameObject go = new GameObject("satie_voice_" + slot);
                go.transform.SetParent(transform, false);
                AudioSource source = go.AddComponent<AudioSource>();
                source.clip = ones;
                source.loop = true;
                source.playOnAwake = false;
                source.dopplerLevel = 0f;
                source.spatialBlend = 1f;
                // With a spatializer plugin, spatialize after the filter, so the plugin sees the voice itself.
                source.spatialize = plugin;
                source.spatializePostEffects = plugin;
                // Level and distance are the core's: Unity's own rolloff is flat.
                source.rolloffMode = AudioRolloffMode.Custom;
                source.SetCustomCurve(AudioSourceCurveType.CustomRolloff, AnimationCurve.Linear(0f, 1f, 1f, 1f));
                SatieVoiceOutput tap = go.AddComponent<SatieVoiceOutput>();
                tap.owner = this;
                tap.slot = slot;
                source.Play();
                voiceNodes[slot] = go.transform;
            }
        }

        // Audio thread. The stream that already holds the current block asks for the next one, so
        // whichever source Unity mixes first in a step renders it for all of them.
        bool TakeBlock(ref long taken, int frames)
        {
            if (taken == renderedBlocks)
            {
                SatieSession session = Session;
                if (session == null || !live || frames > 4096 || !session.RenderObjects(frames)) return false;
                session.CopyAmbient(ambientBlock, frames);
                for (int slot = 0; slot < voiceBlocks.Length; slot++) session.CopyVoice(slot, voiceBlocks[slot], frames);
                if (session.Voices(tableWork))
                {
                    SatieVoice[] swap = tablePublished;
                    tablePublished = tableWork;
                    tableWork = swap;
                }
                Publish(session);
                blockFrames = frames;
                renderedBlocks++;
            }
            taken = renderedBlocks;
            return true;
        }

        internal bool TakeVoice(int slot, ref long taken, float[] mono, int frames)
        {
            lock (blockLock)
            {
                if (slot < 0 || slot >= voiceBlocks.Length || !TakeBlock(ref taken, frames)) return false;
                int usable = Math.Min(frames, blockFrames);
                Array.Copy(voiceBlocks[slot], mono, usable);
                if (usable < frames) Array.Clear(mono, usable, frames - usable);
                return true;
            }
        }

        public void Bind(string host, Transform target) { if (!string.IsNullOrEmpty(host) && target != null) bound[host] = target; }
        public void Unbind(string host) { bound.Remove(host); }

        void BindByName()
        {
            if (Session == null) return;
            foreach (SatieHostEntry h in Session.Manifest.hostList)
            {
                if (bound.ContainsKey(h.id)) continue;
                GameObject go = GameObject.Find(string.IsNullOrEmpty(h.@object) ? h.id : h.@object);
                if (go != null) bound[h.id] = go.transform;
            }
        }

        Transform Ears()
        {
            if (listener != null) return listener;
#pragma warning disable 0618
            AudioListener found = UnityEngine.Object.FindObjectOfType<AudioListener>();
#pragma warning restore 0618
            if (found != null) listener = found.transform;
            else if (Camera.main != null) listener = Camera.main.transform;
            return listener;
        }

        public int Fire(string eventName, string host, string surface, bool hasPosition, Vector3 position)
        {
            if (Session == null) return 0;
            return Session.Event(eventName, host, surface, hasPosition ? new[] { position.x, position.y, position.z } : null);
        }

        void Update()
        {
            if (Session == null) return;
            SendMonitor();
            if (bindTries < 20 && Time.time >= nextBindTry)
            {
                nextBindTry = Time.time + 1f;
                bindTries++;
                BindByName();
                if (bindTries == 20)
                    foreach (SatieHostEntry h in Session.Manifest.hostList)
                        if (!bound.ContainsKey(h.id) && warned.Add(h.id))
                            Debug.LogWarning("Satie: no object found for host " + h.id + " (" + h.@object + "). Its sounds play at the manifest position. Rename the object or call SatieCore.Bind.");
            }
            Transform ears = Ears();
            if (ears != null)
            {
                Vector3 p = ears.position, r = ears.right;
                Session.Listener(p.x, p.y, p.z, r.x, r.y, r.z);
            }
            string gone = null;
            foreach (KeyValuePair<string, Transform> pair in bound)
            {
                if (pair.Value == null) { gone = pair.Key; continue; }
                if (!Session.HasHost(pair.Key)) continue;
                Vector3 p = pair.Value.position;
                Session.MoveHost(pair.Key, p.x, p.y, p.z);
            }
            // A bound object was destroyed (a scene change): forget it and look for its name again.
            if (gone != null) { bound.Remove(gone); warned.Remove(gone); bindTries = 0; }
            // Object output: put each slot's source where the core says its sound is.
            SatieVoice[] table = tablePublished;
            for (int slot = 0; slot < voiceNodes.Length && slot < table.Length; slot++)
                if (table[slot].active != 0 && table[slot].flat == 0)
                    voiceNodes[slot].position = new Vector3(table[slot].x, table[slot].y, table[slot].z);
        }

        // The audio thread: the core's one consumer.
        void OnAudioFilterRead(float[] data, int channels)
        {
            SatieSession session = Session;
            if (!live || session == null || channels < 1) return;
            int frames = data.Length / channels;
            if (voiceNodes.Length > 0)
            {
                // Object output: this source carries the ambient stream (beds and flat voices).
                bool have;
                lock (blockLock) { have = TakeBlock(ref ambientTaken, frames); }
                for (int f = 0; f < frames; f++)
                {
                    bool inBlock = have && f < blockFrames;
                    float l = inBlock ? ambientBlock[2 * f] * masterGain : 0f, r = inBlock ? ambientBlock[2 * f + 1] * masterGain : 0f;
                    if (channels == 1) data[f] = (l + r) * 0.5f;
                    else { data[f * channels] = l; data[f * channels + 1] = r; for (int c = 2; c < channels; c++) data[f * channels + c] = 0f; }
                }
                return;
            }
            if (channels == 2)
            {
                bool rendered;
                lock (blockLock) { session = Session; rendered = session != null && session.Render(data, frames); if (rendered) Publish(session); }
                if (!rendered) Array.Clear(data, 0, data.Length);
                else if (masterGain != 1f) for (int i = 0; i < data.Length; i++) data[i] *= masterGain;
                return;
            }
            if (block.Length != frames * 2) block = new float[frames * 2];
            bool ok;
            lock (blockLock) { session = Session; ok = session != null && session.Render(block, frames); if (ok) Publish(session); }
            for (int f = 0; f < frames; f++)
            {
                float l = ok ? block[2 * f] * masterGain : 0f, r = ok ? block[2 * f + 1] * masterGain : 0f;
                if (channels == 1) data[f] = (l + r) * 0.5f;
                else { data[f * channels] = l; data[f * channels + 1] = r; for (int c = 2; c < channels; c++) data[f * channels + c] = 0f; }
            }
        }
    }
}
