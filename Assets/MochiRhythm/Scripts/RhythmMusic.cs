using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Networking;

namespace MochiRhythm
{
    /// <summary>
    /// The song (port of the three.js version's src/rhythm/music.js): its beat is found once on a
    /// worker thread, then it plays on a loop. SongTime() is the song time being heard now (the
    /// output buffer latency subtracted), counted continuously across loops, plus a calibration the
    /// player can nudge (kept in PlayerPrefs).
    ///
    /// Songs: every AudioClip in Resources/MochiRhythm/Music (imported with Preload Audio Data off,
    /// so only the chosen song is decoded), plus audio files in a "Music" folder under
    /// Application.persistentDataPath (on Quest: /sdcard/Android/data/&lt;package&gt;/files/Music).
    /// </summary>
    [DefaultExecutionOrder(-120)] // before the partner moves the kine and anything else reads SongTime()
    public class RhythmMusic : MonoBehaviour
    {
        const string CalibrationKey = "mochi.rhythm.calibrationMs";
        static readonly string[] Extensions = { ".mp3", ".wav", ".ogg", ".aif", ".aiff" };

        public List<AudioClip> songs = new List<AudioClip>();
        public string folderName = "Music";
        [Range(0f, 1f)] public float volume = 0.8f;

        class Entry { public string name; public AudioClip clip; public string path; public BeatAnalysis analysis; }
        readonly List<Entry> entries = new List<Entry>();
        AudioSource source;
        double dspStart;              // dsp time at which the first sample of the first loop is mixed
        double smooth;                // heard time, smoothed between audio-buffer ticks
        bool smoothValid;
        double pausedAt = double.NaN; // heard time when paused

        public BeatAnalysis Analysis { get; private set; }
        public string SongName { get; private set; } = "";
        public int SongIndex { get; private set; } = -1;
        public int SongCount => entries.Count;
        public bool IsPlaying { get; private set; }
        public bool IsLoading { get; private set; }
        public string Status { get; private set; } = "";
        public int CalibrationMs { get; private set; }
        public double Duration => Analysis != null ? Analysis.Duration : 0;
        public string NameOf(int i) => i >= 0 && i < entries.Count ? entries[i].name : "";

        void Awake()
        {
            source = gameObject.AddComponent<AudioSource>();
            source.playOnAwake = false;
            source.loop = true;
            source.spatialBlend = 0f;
            source.volume = volume;
            // Highest priority: Satie keeps up to 64 voice sources playing and Unity mixes only the project's
            // real voice count (32); lower-priority sources beyond that are virtualized and play silently.
            source.priority = 0;
            CalibrationMs = PlayerPrefs.GetInt(CalibrationKey, 0);
            foreach (var c in songs) if (c != null) entries.Add(new Entry { name = c.name, clip = c });
            AddFolder(Path.Combine(Application.persistentDataPath, folderName));
#if !UNITY_ANDROID || UNITY_EDITOR
            AddFolder(Path.Combine(Application.streamingAssetsPath, folderName)); // inside the APK on Android, not listable
#endif
            Status = entries.Count == 0 ? "No songs. Add audio files to Assets/MochiRhythm/Resources/MochiRhythm/Music." : entries.Count + " songs";
        }

        void AddFolder(string dir)
        {
            try
            {
                if (!Directory.Exists(dir)) return;
                var files = Directory.GetFiles(dir);
                Array.Sort(files, StringComparer.OrdinalIgnoreCase);
                foreach (var f in files)
                {
                    if (Array.IndexOf(Extensions, Path.GetExtension(f).ToLowerInvariant()) < 0) continue;
                    entries.Add(new Entry { name = Path.GetFileNameWithoutExtension(f), path = f });
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning("[rhythm] could not list " + dir + ": " + e.Message);
            }
        }

        /// <summary>Loads and beat-tracks a song (wraps around the list). done(true) once it can play.</summary>
        public void Select(int index, Action<bool> done = null)
        {
            if (entries.Count == 0) { done?.Invoke(false); return; }
            index = ((index % entries.Count) + entries.Count) % entries.Count;
            StopAllCoroutines();
            StartCoroutine(Load(index, done));
        }

        IEnumerator Load(int index, Action<bool> done)
        {
            Stop();
            IsLoading = true;
            var e = entries[index];
            Status = "Loading " + e.name + "...";

            // free the previous song's decoded audio (a few tens of MB on the headset)
            if (source.clip != null && source.clip != e.clip) { source.clip.UnloadAudioData(); source.clip = null; }

            if (e.clip == null && e.path != null)
            {
                using (var req = UnityWebRequestMultimedia.GetAudioClip(new Uri(e.path).AbsoluteUri, AudioTypeOf(e.path)))
                {
                    var handler = (DownloadHandlerAudioClip)req.downloadHandler;
                    handler.streamAudio = false;
                    handler.compressed = false; // decoded into memory, so GetData can read it
                    yield return req.SendWebRequest();
                    if (req.result == UnityWebRequest.Result.Success)
                    {
                        e.clip = handler.audioClip;
                        if (e.clip != null) e.clip.name = e.name;
                    }
                    else Debug.LogWarning("[rhythm] could not load " + e.path + ": " + req.error);
                }
                if (e.clip == null) { Fail("Could not load " + e.name, done); yield break; }
            }

            var clip = e.clip;
            if (clip.loadState != AudioDataLoadState.Loaded)
            {
                clip.LoadAudioData();
                float give = Time.realtimeSinceStartup + 30f;
                while (clip.loadState != AudioDataLoadState.Loaded && clip.loadState != AudioDataLoadState.Failed && Time.realtimeSinceStartup < give)
                    yield return null;
            }
            if (clip.loadState != AudioDataLoadState.Loaded) { Fail("Could not decode " + e.name, done); yield break; }

            if (e.analysis == null)
            {
                Status = "Finding the beat in " + e.name + "...";
                int channels = Mathf.Max(1, clip.channels), total = clip.samples, rate = clip.frequency;
                var mono = new float[total];
                const int Chunk = 1 << 16;
                var buf = new float[Chunk * channels];
                for (int off = 0, step = 0; off < total; off += Chunk, step++)
                {
                    int n = Math.Min(Chunk, total - off);
                    if (n != Chunk) buf = new float[n * channels]; // GetData wraps around past the end
                    if (!clip.GetData(buf, off))
                    {
                        Fail("Can't read the samples of " + e.name + " (its Load Type must be Decompress On Load)", done);
                        yield break;
                    }
                    for (int i = 0; i < n; i++)
                    {
                        float s = 0f;
                        for (int c = 0; c < channels; c++) s += buf[i * channels + c];
                        mono[off + i] = s / channels;
                    }
                    if (step % 16 == 15) yield return null; // keep frames coming on the headset
                }
                var task = Task.Run(() => BeatAnalyzer.Analyze(mono, rate));
                while (!task.IsCompleted) yield return null;
                if (task.IsFaulted || task.Result == null)
                {
                    string why = task.Exception != null ? task.Exception.GetBaseException().Message : "no result";
                    Fail("No steady beat found in " + e.name + " (" + why + ")", done);
                    yield break;
                }
                if (task.Result.Beats.Length < 8) { Fail("No steady beat found in " + e.name, done); yield break; }
                e.analysis = task.Result;
            }

            source.clip = clip;
            Analysis = e.analysis;
            SongName = e.name;
            SongIndex = index;
            Status = string.Format("{0}: {1:0} BPM", e.name, Analysis.Bpm);
            Debug.Log(string.Format("[rhythm] {0}: {1:0.0} BPM, {2} beats over {3:0} s", e.name, Analysis.Bpm, Analysis.Beats.Length, Analysis.Duration));
            IsLoading = false;
            done?.Invoke(true);
        }

        void Fail(string message, Action<bool> done)
        {
            Debug.LogWarning("[rhythm] " + message);
            Status = message;
            IsLoading = false;
            done?.Invoke(false);
        }

        static AudioType AudioTypeOf(string path)
        {
            switch (Path.GetExtension(path).ToLowerInvariant())
            {
                case ".mp3": return AudioType.MPEG;
                case ".wav": return AudioType.WAV;
                case ".ogg": return AudioType.OGGVORBIS;
                case ".aif":
                case ".aiff": return AudioType.AIFF;
                default: return AudioType.UNKNOWN;
            }
        }

        /// <summary>Starts the loaded song from the top, on a loop, 0.25 s from now.</summary>
        public bool Play()
        {
            if (source.clip == null || Analysis == null || IsLoading) return false;
            source.Stop();
            source.timeSamples = 0;
            dspStart = AudioSettings.dspTime + 0.25;
            source.PlayScheduled(dspStart);
            IsPlaying = true;
            smoothValid = false;
            pausedAt = double.NaN;
            return true;
        }

        /// <summary>Shows a message on the panel's status line.</summary>
        public void SetStatus(string message) => Status = message;

        float fade = 1f;
        Coroutine fading;

        /// <summary>Fades the song out over the given time, then stops it.</summary>
        public void FadeOutAndStop(float seconds)
        {
            if (!IsPlaying || fading != null) return;
            fading = StartCoroutine(FadeOut(Mathf.Max(0.05f, seconds)));
        }

        IEnumerator FadeOut(float seconds)
        {
            for (float t = 0f; t < seconds; t += Time.unscaledDeltaTime) { fade = 1f - t / seconds; yield return null; }
            fading = null;
            Stop();
        }

        public void Stop()
        {
            if (fading != null) { StopCoroutine(fading); fading = null; }
            fade = 1f;
            if (source != null) source.Stop();
            IsPlaying = false;
            pausedAt = double.NaN;
        }

        public void Pause()
        {
            if (!IsPlaying || !double.IsNaN(pausedAt)) return;
            pausedAt = Math.Max(0, HeardTime());
            source.Pause();
        }

        public void Resume()
        {
            if (!IsPlaying || double.IsNaN(pausedAt) || source.clip == null) return;
            double dur = Duration;
            double pos = dur > 0 ? pausedAt % dur : 0;
            source.Stop();
            source.timeSamples = Mathf.Clamp((int)(pos * source.clip.frequency), 0, source.clip.samples - 1);
            double startAt = AudioSettings.dspTime + 0.2;
            source.PlayScheduled(startAt);
            dspStart = startAt - pausedAt; // so the heard time continues from where it stopped
            pausedAt = double.NaN;
            smoothValid = false;
        }

        void OnApplicationPause(bool paused)
        {
            if (paused) Pause();
            else Resume();
        }

#if !UNITY_EDITOR
        // on the headset, losing focus means the system menu is open
        void OnApplicationFocus(bool focused)
        {
            if (!focused) Pause();
            else Resume();
        }
#endif

        /// <summary>Song time being heard, seconds since the song started, continuous across loops.</summary>
        public double SongTime()
        {
            if (!IsPlaying) return double.NegativeInfinity;
            double t = !double.IsNaN(pausedAt) ? pausedAt : HeardTime();
            return t + CalibrationMs / 1000.0;
        }

        double HeardTime() => smoothValid ? smooth : AudioSettings.dspTime - dspStart - OutputLatency();

        void Update()
        {
            if (source != null) source.volume = volume * fade;
            if (!IsPlaying || !double.IsNaN(pausedAt)) return;
            // dspTime only moves once per audio buffer (~10-20 ms): advance with the frame clock and
            // lean gently toward it, snapping if they drift apart
            double raw = AudioSettings.dspTime - dspStart - OutputLatency();
            if (!smoothValid || Math.Abs(raw - smooth) > 0.08)
            {
                smooth = raw;
                smoothValid = true;
            }
            else
            {
                smooth += Time.unscaledDeltaTime;
                smooth += (raw - smooth) * 0.05;
            }
        }

        // Unity mixes audio a few buffers ahead of the speakers; the player's calibration covers the rest.
        static double OutputLatency()
        {
            AudioSettings.GetDSPBufferSize(out int length, out int count);
            int rate = AudioSettings.outputSampleRate;
            if (rate <= 0) return 0;
            return Math.Min(0.15, (double)length * Math.Max(1, count) / rate);
        }

        public int Nudge(int ms)
        {
            CalibrationMs = Mathf.Clamp(CalibrationMs + ms, -300, 300);
            PlayerPrefs.SetInt(CalibrationKey, CalibrationMs);
            PlayerPrefs.Save();
            return CalibrationMs;
        }
    }
}
