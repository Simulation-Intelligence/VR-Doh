// SatieCore.cs: the portable SATIE core from C#. No UnityEngine types here, so the
// same file is compiled and run by the .NET conformance test (tests/Conformance).
// SatieCoreBehaviour.cs is the Unity side: files, the audio callback, transforms.
//
// Threading is the core's contract (docs/contract.md): exactly one thread calls
// the methods that enqueue (the game's main thread), exactly one calls Render
// (the audio callback). The session adds no locks.
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text;
using System.Threading;

namespace Satie.Core
{
    [StructLayout(LayoutKind.Sequential)]
    public struct SatieCommand
    {
        public ulong frame;
        public uint kind, source, slot, sequence;
        public float v0, v1, v2, v3, v4, v5;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SatieStats
    {
        public ulong frame;
        public uint activeVoices, lateCommands, duplicateEvents, droppedVoices, clippedSamples;
    }

    /// One slot of the voice table in object output. A slot is a stable index.
    [StructLayout(LayoutKind.Sequential)]
    public struct SatieVoice
    {
        public uint active, generation, source, flat;
        public float x, y, z, reserved;
    }

    public static class SatieNative
    {
#if UNITY_IOS && !UNITY_EDITOR
        const string Library = "__Internal";
#else
        const string Library = "satie_core";
#endif
        public const uint Trigger = 1, Position = 2, Listener = 3, Control = 4, StopSource = 5, StartSource = 6, State = 7, Monitor = 8;

        [DllImport(Library)] public static extern uint satie_abi_version();
        [DllImport(Library)] public static extern IntPtr satie_create(byte[] bundle, UIntPtr size, uint outputSampleRate, uint maxVoices, out int result);
        [DllImport(Library)] public static extern void satie_destroy(IntPtr runtime);
        [DllImport(Library)] public static extern uint satie_source_count(IntPtr runtime);
        [DllImport(Library)] public static extern uint satie_latency_frames(IntPtr runtime);
        [DllImport(Library)] public static extern int satie_enqueue(IntPtr runtime, ref SatieCommand command);
        [DllImport(Library)] public static extern int satie_render(IntPtr runtime, [In, Out] float[] stereo, uint frames);
        [DllImport(Library)] public static extern int satie_get_stats(IntPtr runtime, out SatieStats stats);
        [DllImport(Library)] public static extern int satie_enable_objects(IntPtr runtime);
        [DllImport(Library)] public static extern uint satie_voice_capacity(IntPtr runtime);
        [DllImport(Library)] public static extern int satie_render_objects(IntPtr runtime, uint frames);
        [DllImport(Library)] public static extern IntPtr satie_voice_block(IntPtr runtime, uint slot);
        [DllImport(Library)] public static extern IntPtr satie_ambient_block(IntPtr runtime);
        [DllImport(Library)] public static extern int satie_get_voices(IntPtr runtime, [In, Out] SatieVoice[] voices, uint capacity);
        [DllImport(Library)] public static extern int satie_get_levels(IntPtr runtime, [In, Out] float[] peaks, uint capacity);
    }

    // The manifest (satie.portable.bundle/2), as the fields a host needs. Public
    // fields and arrays only: Unity's JsonUtility reads nothing else.
    [Serializable] public class SatieAnswer { public int source; public string host = ""; public string[] surfaces = new string[0]; }
    [Serializable] public class SatieEventEntry { public string name = ""; public SatieAnswer[] answers = new SatieAnswer[0]; }
    [Serializable] public class SatieBinding { public int source, slot; }
    [Serializable] public class SatieSignalEntry { public string name = ""; public float rest; public SatieBinding[] bindings = new SatieBinding[0]; }
    [Serializable] public class SatieHostEntry { public string id = "", @object = ""; public bool hasPosition; public float[] position = new float[3]; public int[] sources = new int[0]; }
    [Serializable] public class SatieSourceEntry { public int index; public string id = "", kind = "", host = "", @event = ""; public bool atListener, spatial; }
    [Serializable]
    public class SatieManifest
    {
        public string schema = "", profile = "", bundleSha256 = "", name = "";
        public int abi, sampleRate;
        public SatieSourceEntry[] sources = new SatieSourceEntry[0];
        public string[] states = new string[0];
        public SatieEventEntry[] eventList = new SatieEventEntry[0];
        public SatieSignalEntry[] signalList = new SatieSignalEntry[0];
        public SatieHostEntry[] hostList = new SatieHostEntry[0];
        public string[] notRendered = new string[0];
    }

    /// One loaded scene. The host's five ideas by name: Event, Signal, State,
    /// MoveHost and Listener, exactly as compiler/session.ts and Godot's SatieWorld
    /// issue them (tests/host_script.json holds all three to the same commands).
    public sealed class SatieSession : IDisposable
    {
        IntPtr runtime;
        readonly SatieManifest manifest;
        readonly Dictionary<string, SatieEventEntry> events = new Dictionary<string, SatieEventEntry>();
        readonly Dictionary<string, SatieSignalEntry> signals = new Dictionary<string, SatieSignalEntry>();
        readonly Dictionary<string, SatieHostEntry> hosts = new Dictionary<string, SatieHostEntry>();
        readonly Dictionary<string, float[]> hostAt = new Dictionary<string, float[]>();
        readonly Dictionary<string, string> hostSurface = new Dictionary<string, string>();
        readonly Dictionary<int, float[]> sourceAt = new Dictionary<int, float[]>();
        readonly Dictionary<int, uint> sequence = new Dictionary<int, uint>();
        float[] ears = new float[3];
        long rendered;
        long lastFrame;

        /// Commands the core refused (a full queue, an unknown index). A host shows this; it does not hide it.
        public int Rejected { get; private set; }
        /// When set, every command is stamped with this frame instead of the audio clock (tests and offline renders).
        public long? FrameOverride;
        /// When set, every accepted command is also appended here.
        public List<SatieCommand> Trace;
        public SatieManifest Manifest { get { return manifest; } }
        public int OutputRate { get; private set; }
        /// Frames rendered so far: the audio clock commands are stamped against.
        public long Frame { get { return Interlocked.Read(ref rendered); } }
        public int LatencyFrames { get { return runtime == IntPtr.Zero ? 0 : (int)SatieNative.satie_latency_frames(runtime); } }

        public SatieSession(byte[] bundle, SatieManifest manifest, int outputRate, int maxVoices = 64)
        {
            if (bundle == null || manifest == null) throw new ArgumentNullException();
            if (manifest.schema != "satie.portable.bundle/2" || manifest.profile != "satie.portable.scene/2" || manifest.abi != 2)
                throw new InvalidOperationException("Satie: not a scene-profile manifest (" + manifest.schema + ", " + manifest.profile + ")");
            uint abi = SatieNative.satie_abi_version();
            if (abi < 3) throw new InvalidOperationException("Satie: the native library is older than ABI 3");
            CanMonitor = abi >= 4;
            // The manifest's indices are only meaningful for the exact bundle it was written with.
            if (!string.Equals(Sha256(bundle), manifest.bundleSha256, StringComparison.OrdinalIgnoreCase))
                throw new InvalidOperationException("Satie: the bundle does not match its manifest (hash differs)");
            int result;
            runtime = SatieNative.satie_create(bundle, (UIntPtr)bundle.Length, (uint)outputRate, (uint)maxVoices, out result);
            if (runtime == IntPtr.Zero) throw new InvalidOperationException("Satie: the core refused the bundle (result " + result + ")");
            if (SatieNative.satie_source_count(runtime) != manifest.sources.Length)
            {
                Dispose();
                throw new InvalidOperationException("Satie: source count differs between bundle and manifest");
            }
            this.manifest = manifest;
            OutputRate = outputRate;
            foreach (SatieEventEntry e in manifest.eventList) events[e.name] = e;
            foreach (SatieSignalEntry s in manifest.signalList) signals[s.name] = s;
            foreach (SatieHostEntry h in manifest.hostList)
            {
                hosts[h.id] = h;
                if (!h.hasPosition) continue;
                hostAt[h.id] = h.position;
                foreach (int s in h.sources) sourceAt[s] = h.position;
            }
        }

        static string Sha256(byte[] bytes)
        {
            using (SHA256 sha = SHA256.Create())
            {
                StringBuilder hex = new StringBuilder(64);
                foreach (byte b in sha.ComputeHash(bytes)) hex.Append(b.ToString("x2"));
                return hex.ToString();
            }
        }

        long Stamp()
        {
            if (FrameOverride.HasValue) return FrameOverride.Value;
            // The next block boundary. A command that arrives after it is applied at once and counted late by the core.
            lastFrame = Math.Max(lastFrame, Frame);
            return lastFrame;
        }

        bool Send(uint kind, int source, int slot, uint seq, float v0 = 0, float v1 = 0, float v2 = 0, float v3 = 0, float v4 = 0, float v5 = 0)
        {
            if (runtime == IntPtr.Zero) return false;
            SatieCommand c = new SatieCommand { frame = (ulong)Stamp(), kind = kind, source = (uint)source, slot = (uint)slot, sequence = seq, v0 = v0, v1 = v1, v2 = v2, v3 = v3, v4 = v4, v5 = v5 };
            if (SatieNative.satie_enqueue(runtime, ref c) != 0) { Rejected++; return false; }
            if (Trace != null) Trace.Add(c);
            return true;
        }

        /// The ears: position and the unit vector pointing to the listener's right.
        public void Listener(float x, float y, float z, float rightX = 1, float rightY = 0, float rightZ = 0)
        {
            ears = new[] { x, y, z };
            Send(SatieNative.Listener, 0, 0, 0, x, y, z, rightX, rightY, rightZ);
            PublishDistance();
        }

        public bool HasHost(string host) { return host != null && hosts.ContainsKey(host); }

        public void MoveHost(string host, float x, float y, float z)
        {
            SatieHostEntry h;
            if (host == null || !hosts.TryGetValue(host, out h)) throw new ArgumentException("Unknown host " + host);
            float[] p = { x, y, z };
            hostAt[host] = p;
            foreach (int source in h.sources) { sourceAt[source] = p; Send(SatieNative.Position, source, 0, 0, x, y, z); }
            PublishDistance();
        }

        /// The material under a host now. Set it before the event that reads it.
        public void SetHostSurface(string host, string surface) { hostSurface[host] = (surface ?? "").ToLowerInvariant(); }

        /// A continuous control: every bound source receives the raw value. False for a signal the scene does not follow.
        public bool Signal(string name, float value)
        {
            SatieSignalEntry s;
            if (name == null || !signals.TryGetValue(name, out s)) return false;
            foreach (SatieBinding b in s.bindings) Send(SatieNative.Control, b.source, b.slot, 0, value);
            return true;
        }

        /// True when the native library has the mixer's calls (ABI 4): Monitor and Levels.
        public bool CanMonitor { get; private set; }

        /// A mixer's audition gain on everything one source plays (0 to 4, 1 = as written). Not part of the scene.
        public bool Monitor(int source, float gain)
        {
            if (!CanMonitor || source < 0 || source >= manifest.sources.Length) return false;
            return Send(SatieNative.Monitor, source, 0, 0, Math.Min(4f, Math.Max(0f, gain)));
        }

        /// Audio thread: the peak each source reached in the last render call, linear, one per manifest source.
        public bool Levels(float[] peaks)
        {
            return CanMonitor && runtime != IntPtr.Zero && peaks != null && peaks.Length >= manifest.sources.Length
                && SatieNative.satie_get_levels(runtime, peaks, (uint)peaks.Length) == 0;
        }

        /// Where a host is now: the manifest's place until the game moves it. False for a host with no place.
        public bool HostPosition(string host, out float x, out float y, out float z)
        {
            float[] p;
            x = y = z = 0;
            if (host == null || !hostAt.TryGetValue(host, out p)) return false;
            x = p[0]; y = p[1]; z = p[2];
            return true;
        }

        /// The mode the game is in; null or empty for none.
        public bool State(string name)
        {
            int index = string.IsNullOrEmpty(name) ? -1 : Array.IndexOf(manifest.states, name);
            if (!string.IsNullOrEmpty(name) && index < 0) return false;
            return Send(SatieNative.State, 0, index + 1, 0);
        }

        /// An event: from a host (its voices and hostless ones answer, at the host), or at a place, or at the ears.
        /// Returns how many voices were sent a trigger.
        public int Event(string name, string host = null, string surface = null, float[] position = null)
        {
            SatieEventEntry e;
            if (name == null || !events.TryGetValue(name, out e)) return 0;
            string known;
            string surf = !string.IsNullOrEmpty(surface) ? surface.ToLowerInvariant()
                : (!string.IsNullOrEmpty(host) && hostSurface.TryGetValue(host, out known) ? known : null);
            int fired = 0;
            foreach (SatieAnswer a in e.answers)
            {
                if (!string.IsNullOrEmpty(host) && !string.IsNullOrEmpty(a.host) && a.host != host) continue;
                float[] where = position;
                if (where == null)
                {
                    string from = !string.IsNullOrEmpty(a.host) ? a.host : host;
                    if (string.IsNullOrEmpty(from) || !hostAt.TryGetValue(from, out where)) where = ears;
                }
                int bank = string.IsNullOrEmpty(surf) ? 0 : Array.IndexOf(a.surfaces, surf) + 1;
                uint seq;
                sequence.TryGetValue(a.source, out seq);
                sequence[a.source] = ++seq;
                if (Send(SatieNative.Trigger, a.source, bank, seq, where[0], where[1], where[2])) fired++;
            }
            return fired;
        }

        void PublishDistance()
        {
            SatieSignalEntry d;
            if (!signals.TryGetValue("distance", out d)) return;
            foreach (SatieBinding b in d.bindings)
            {
                float[] p;
                if (!sourceAt.TryGetValue(b.source, out p)) continue;
                float dx = p[0] - ears[0], dy = p[1] - ears[1], dz = p[2] - ears[2];
                Send(SatieNative.Control, b.source, b.slot, 0, (float)Math.Sqrt(dx * dx + dy * dy + dz * dz));
            }
        }

        /// The audio callback: interleaved stereo, any number of frames. Allocates nothing after the first call of a given size.
        public bool Render(float[] stereo, int frames)
        {
            if (runtime == IntPtr.Zero || stereo == null || frames <= 0 || stereo.Length < frames * 2) return false;
            if (frames <= 4096 && stereo.Length == frames * 2)
            {
                if (SatieNative.satie_render(runtime, stereo, (uint)frames) != 0) return false;
                Interlocked.Add(ref rendered, frames);
                return true;
            }
            // The core takes at most 4096 frames a call and writes from the start of the array it is given.
            for (int done = 0; done < frames;)
            {
                int n = Math.Min(4096, frames - done);
                if (scratch == null || scratch.Length != n * 2) scratch = new float[n * 2];
                if (SatieNative.satie_render(runtime, scratch, (uint)n) != 0) return false;
                Array.Copy(scratch, 0, stereo, done * 2, n * 2);
                Interlocked.Add(ref rendered, n);
                done += n;
            }
            return true;
        }
        float[] scratch;

        // ── Object output: the core decides what sounds and how loud, the engine gives each voice its direction ──

        /// Choose object output. Call once, off the audio thread, before the first render.
        public bool EnableObjects()
        {
            if (runtime == IntPtr.Zero || SatieNative.satie_enable_objects(runtime) != 0) return false;
            VoiceSlots = (int)SatieNative.satie_voice_capacity(runtime);
            return true;
        }
        /// Voice slots in object output (0 until EnableObjects). One engine audio source per slot.
        public int VoiceSlots { get; private set; }

        /// Audio thread: render the next block (1 to 4096 frames) into the core's own streams.
        public bool RenderObjects(int frames)
        {
            if (runtime == IntPtr.Zero || VoiceSlots == 0 || frames <= 0 || frames > 4096) return false;
            if (SatieNative.satie_render_objects(runtime, (uint)frames) != 0) return false;
            Interlocked.Add(ref rendered, frames);
            return true;
        }
        /// Audio thread: the last block of one voice slot, mono, already levelled and distance-attenuated.
        public void CopyVoice(int slot, float[] mono, int frames) { Marshal.Copy(SatieNative.satie_voice_block(runtime, (uint)slot), mono, 0, frames); }
        /// Audio thread: the last block of the ambient stream (beds and flat voices), interleaved stereo.
        public void CopyAmbient(float[] stereo, int frames) { Marshal.Copy(SatieNative.satie_ambient_block(runtime), stereo, 0, frames * 2); }
        /// Audio thread: which sound each slot holds and where it is.
        public bool Voices(SatieVoice[] table) { return runtime != IntPtr.Zero && table != null && SatieNative.satie_get_voices(runtime, table, (uint)table.Length) == 0; }

        /// Audio thread only, or after it has stopped.
        public SatieStats Stats()
        {
            SatieStats s;
            if (runtime == IntPtr.Zero) return new SatieStats();
            SatieNative.satie_get_stats(runtime, out s);
            return s;
        }

        /// Stop the audio callback before disposing: the core has no concurrent destroy.
        public void Dispose()
        {
            if (runtime == IntPtr.Zero) return;
            SatieNative.satie_destroy(runtime);
            runtime = IntPtr.Zero;
        }
    }
}
