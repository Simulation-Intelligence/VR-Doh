// SatieMixerWindow.cs: the Satie mixer inside the Unity editor (Window > Satie > Mixer).
//
// A view, nothing more. It reads the scene's manifest and the running core
// (SatieCoreBehaviour): one row per sound with its level, a volume, mute and solo;
// the scene's signals, states and events; a dot per sound in the Scene view. What
// the developer decides is auditioned through the core's monitor gain and, on Save,
// appended to .satie/authoring/edits.jsonl as satie.edits/v1 lines (the same log
// the browser mixer writes, packages/satie-runtime/EDITS.md). The script changes
// where it always does: satie_apply_edits reads that log. When the compiled scene
// files change on disk during Play, the window reloads them into the running core.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

namespace Satie.Core.EditorTools
{
    public class SatieMixerWindow : EditorWindow
    {
        // The browser mixer's voice colours on a dark ground (src/engine/devtools/index.ts).
        static readonly Color[] VoiceColors = { Hex("ff8da1"), Hex("ff9db0"), Hex("f7889c"), Hex("ffadbd"), Hex("fb93a6") };
        static readonly Color Rose = Hex("ff8da1"), Ink = new Color(1f, 1f, 1f, 0.92f), Dim = new Color(1f, 1f, 1f, 0.5f),
            Faint = new Color(1f, 1f, 1f, 0.08f), Ground = new Color(0.07f, 0.07f, 0.075f, 1f), Warn = Hex("ffcf6b");
        const string BundlePath = "Assets/Resources/SatieCore/scene.satp.bytes", ManifestPath = "Assets/Resources/SatieCore/scene.manifest.json";
        const float FloorDb = -60f;

        class Row
        {
            public int source;
            public SatieSourceEntry entry;
            public VisualElement root, meter;
            public Slider volume;
            public Label amount;
            public Button mute, solo;
            public float shownDb = FloorDb;
        }

        SatieManifest manifest;
        SatieCoreBehaviour core;
        int loadsSeen = -1;
        readonly List<Row> rows = new List<Row>();
        readonly Dictionary<string, Slider> signalSliders = new Dictionary<string, Slider>();
        readonly Dictionary<string, Button> stateButtons = new Dictionary<string, Button>();
        // What Save has already written, per sound id, so a second Save writes only what changed since.
        readonly Dictionary<string, float> savedTrim = new Dictionary<string, float>();
        readonly Dictionary<string, bool> savedMute = new Dictionary<string, bool>();
        readonly Dictionary<string, Vector3> movedHosts = new Dictionary<string, Vector3>();
        Label heading, status, counters, changes;
        Button save, discard, direction;
        VisualElement body;
        int selected = -1;
        DateTime bundleStamp, manifestStamp;
        double nextFileCheck;

        [MenuItem("Window/Satie/Mixer")]
        public static void Open()
        {
            SatieMixerWindow w = GetWindow<SatieMixerWindow>();
            w.titleContent = new GUIContent("Satie");
            w.minSize = new Vector2(340, 320);
        }

        static Color Hex(string hex)
        {
            Color c;
            ColorUtility.TryParseHtmlString("#" + hex, out c);
            return c;
        }

        void OnEnable()
        {
            SceneView.duringSceneGui += OnScene;
            EditorApplication.playModeStateChanged += OnPlayMode;
            Build();
        }

        void OnDisable()
        {
            SceneView.duringSceneGui -= OnScene;
            EditorApplication.playModeStateChanged -= OnPlayMode;
        }

        void OnPlayMode(PlayModeStateChange change) { core = null; loadsSeen = -1; Build(); }

        // ── The window ──

        void Build()
        {
            VisualElement root = rootVisualElement;
            root.Clear();
            rows.Clear();
            signalSliders.Clear();
            stateButtons.Clear();
            root.style.backgroundColor = Ground;
            root.style.color = Ink;
            root.style.fontSize = 12;

            VisualElement head = new VisualElement();
            head.style.paddingLeft = 14; head.style.paddingRight = 14; head.style.paddingTop = 12; head.style.paddingBottom = 10;
            head.style.borderBottomWidth = 1; head.style.borderBottomColor = Faint;
            head.style.flexShrink = 0; // the list scrolls; the head and the foot keep their height
            heading = new Label("Satie");
            heading.style.fontSize = 16; heading.style.unityFontStyleAndWeight = FontStyle.Bold;
            status = new Label();
            status.style.color = Dim; status.style.marginTop = 2; status.style.whiteSpace = WhiteSpace.Normal;
            counters = new Label();
            counters.style.color = Dim; counters.style.marginTop = 2; counters.style.fontSize = 11;
            direction = Pill("", () => { if (SatieCoreDemo.Instance != null) SatieCoreDemo.Instance.SwitchDirection(); });
            direction.style.marginLeft = 0; direction.style.marginTop = 8; direction.style.alignSelf = Align.FlexStart;
            direction.tooltip = "Who gives each sound its direction. Switching starts the scene again on a new core.";
            head.Add(heading); head.Add(status); head.Add(counters); head.Add(direction);
            root.Add(head);

            ScrollView scroll = new ScrollView(ScrollViewMode.Vertical);
            scroll.style.flexGrow = 1;
            body = scroll.contentContainer;
            body.style.paddingLeft = 14; body.style.paddingRight = 14; body.style.paddingBottom = 12;
            root.Add(scroll);

            VisualElement foot = new VisualElement();
            foot.style.flexDirection = FlexDirection.Row; foot.style.alignItems = Align.Center;
            foot.style.paddingLeft = 14; foot.style.paddingRight = 14; foot.style.paddingTop = 8; foot.style.paddingBottom = 8;
            foot.style.borderTopWidth = 1; foot.style.borderTopColor = Faint;
            foot.style.flexShrink = 0;
            changes = new Label();
            changes.style.flexGrow = 1; changes.style.color = Dim; changes.style.whiteSpace = WhiteSpace.Normal;
            discard = Pill("Discard", Discard);
            save = Pill("Save changes", Save);
            save.style.backgroundColor = Rose; save.style.color = Color.black;
            foot.Add(changes); foot.Add(discard); foot.Add(save);
            root.Add(foot);

            manifest = ReadManifest();
            Fill();
            root.schedule.Execute(Tick).Every(33);
        }

        SatieManifest ReadManifest()
        {
            if (core != null && core.Session != null) return core.Session.Manifest;
            TextAsset json = Resources.Load<TextAsset>("SatieCore/scene.manifest");
            if (json == null) return null;
            try { return JsonUtility.FromJson<SatieManifest>(json.text); }
            catch (Exception) { return null; }
        }

        void Fill()
        {
            body.Clear();
            rows.Clear();
            signalSliders.Clear();
            stateButtons.Clear();
            if (manifest == null)
            {
                heading.text = "Satie";
                status.text = "No compiled scene in this project. Put scene.satp.bytes and scene.manifest.json in Assets/Resources/SatieCore.";
                return;
            }
            heading.text = string.IsNullOrEmpty(manifest.name) ? "Satie" : Words(manifest.name);

            body.Add(Section("Sounds"));
            for (int i = 0; i < manifest.sources.Length; i++) body.Add(BuildRow(i));

            if (manifest.signalList.Length > 0)
            {
                body.Add(Section("Signals"));
                foreach (SatieSignalEntry s in manifest.signalList)
                {
                    if (s.name == "distance") continue; // the component publishes it from positions
                    string name = s.name;
                    Slider slider = new Slider(Words(name), 0f, 1f);
                    slider.showInputField = true;
                    slider.SetValueWithoutNotify(s.rest);
                    slider.labelElement.style.minWidth = 110; slider.labelElement.style.color = Ink;
                    slider.RegisterValueChangedCallback(e => { if (core != null) core.Signal(name, e.newValue); });
                    signalSliders[name] = slider;
                    body.Add(slider);
                }
            }

            if (manifest.states.Length > 0)
            {
                body.Add(Section("State"));
                VisualElement wrap = new VisualElement();
                wrap.style.flexDirection = FlexDirection.Row; wrap.style.flexWrap = Wrap.Wrap;
                wrap.Add(StateButton("none", null));
                foreach (string state in manifest.states) wrap.Add(StateButton(Words(state), state));
                body.Add(wrap);
            }
            Refresh();
        }

        Label Section(string text)
        {
            Label l = new Label(text.ToUpperInvariant());
            l.style.fontSize = 10; l.style.color = Dim; l.style.letterSpacing = 1.5f;
            l.style.marginTop = 16; l.style.marginBottom = 6;
            return l;
        }

        Button Pill(string text, Action click)
        {
            Button b = new Button(click) { text = text };
            b.style.borderTopLeftRadius = 10; b.style.borderTopRightRadius = 10; b.style.borderBottomLeftRadius = 10; b.style.borderBottomRightRadius = 10;
            b.style.borderTopWidth = 0; b.style.borderBottomWidth = 0; b.style.borderLeftWidth = 0; b.style.borderRightWidth = 0;
            b.style.backgroundColor = Faint; b.style.color = Ink;
            b.style.paddingLeft = 10; b.style.paddingRight = 10; b.style.paddingTop = 3; b.style.paddingBottom = 3;
            b.style.marginLeft = 4; b.style.marginRight = 0; b.style.marginTop = 2; b.style.marginBottom = 2;
            return b;
        }

        Button StateButton(string text, string state)
        {
            Button b = Pill(text, () => { if (core != null) { core.State(state); Refresh(); } });
            b.style.marginLeft = 0; b.style.marginRight = 4;
            stateButtons[state ?? ""] = b;
            return b;
        }

        VisualElement BuildRow(int i)
        {
            SatieSourceEntry entry = manifest.sources[i];
            Row row = new Row { source = i, entry = entry };
            Color color = VoiceColors[i % VoiceColors.Length];

            VisualElement root = new VisualElement();
            root.style.paddingTop = 7; root.style.paddingBottom = 7; root.style.paddingLeft = 8; root.style.paddingRight = 6;
            root.style.marginBottom = 2;
            root.style.borderTopLeftRadius = 6; root.style.borderTopRightRadius = 6; root.style.borderBottomLeftRadius = 6; root.style.borderBottomRightRadius = 6;
            root.style.borderLeftWidth = 2; root.style.borderLeftColor = Color.clear;
            root.style.flexShrink = 0;
            root.RegisterCallback<MouseDownEvent>(e => Select(i));

            VisualElement top = new VisualElement();
            top.style.flexDirection = FlexDirection.Row; top.style.alignItems = Align.Center;
            VisualElement dot = new VisualElement();
            dot.style.width = 8; dot.style.height = 8; dot.style.marginRight = 8; dot.style.backgroundColor = color;
            dot.style.borderTopLeftRadius = 4; dot.style.borderTopRightRadius = 4; dot.style.borderBottomLeftRadius = 4; dot.style.borderBottomRightRadius = 4;
            Label name = new Label(Words(entry.id));
            name.style.unityFontStyleAndWeight = FontStyle.Bold; name.style.flexShrink = 1; name.style.overflow = Overflow.Hidden;
            Label where = new Label(Where(entry));
            where.style.color = Dim; where.style.marginLeft = 8; where.style.flexGrow = 1; where.style.fontSize = 11;
            top.Add(dot); top.Add(name); top.Add(where);
            if (entry.kind == "event" && !string.IsNullOrEmpty(entry.@event))
            {
                Button play = Pill("Play", () => FireFor(entry, null));
                play.tooltip = "Fire the event " + entry.@event;
                top.Add(play);
                // One more per surface the sound has takes for (a step on wood, on fabric).
                foreach (SatieEventEntry e in manifest.eventList)
                    if (e.name == entry.@event)
                        foreach (SatieAnswer answer in e.answers)
                            if (answer.source == i)
                                foreach (string surface in answer.surfaces)
                                {
                                    string on = surface;
                                    Button b = Pill(Words(on), () => FireFor(entry, on));
                                    b.tooltip = "Fire " + entry.@event + " on " + on;
                                    top.Add(b);
                                }
            }
            row.mute = Pill("Mute", () => { if (core != null) { core.SetMuted(i, !core.Muted(i)); Refresh(); } });
            row.solo = Pill("Solo", () => { if (core != null) { core.SetSoloed(i, !core.Soloed(i)); Refresh(); } });
            top.Add(row.mute); top.Add(row.solo);
            root.Add(top);

            // The level this sound reaches right now.
            VisualElement track = new VisualElement();
            track.style.height = 3; track.style.marginTop = 7; track.style.marginLeft = 16; track.style.backgroundColor = Faint;
            track.style.borderTopLeftRadius = 2; track.style.borderTopRightRadius = 2; track.style.borderBottomLeftRadius = 2; track.style.borderBottomRightRadius = 2;
            row.meter = new VisualElement();
            row.meter.style.height = 3; row.meter.style.width = Length.Percent(0); row.meter.style.backgroundColor = color;
            row.meter.style.borderTopLeftRadius = 2; row.meter.style.borderTopRightRadius = 2; row.meter.style.borderBottomLeftRadius = 2; row.meter.style.borderBottomRightRadius = 2;
            track.Add(row.meter);
            root.Add(track);

            VisualElement bottom = new VisualElement();
            bottom.style.flexDirection = FlexDirection.Row; bottom.style.alignItems = Align.Center; bottom.style.marginTop = 4; bottom.style.marginLeft = 16;
            Label volume = new Label("Volume");
            volume.style.color = Dim; volume.style.fontSize = 11; volume.style.width = 48;
            row.volume = new Slider(-24f, 12f);
            row.volume.style.flexGrow = 1;
            row.volume.RegisterValueChangedCallback(e =>
            {
                if (core == null) return;
                core.SetTrimDb(i, Mathf.Round(e.newValue)); // whole decibels: the script's level is written in them
                Refresh();
            });
            row.amount = new Label();
            row.amount.style.width = 64; row.amount.style.unityTextAlign = TextAnchor.MiddleRight; row.amount.style.fontSize = 11;
            bottom.Add(volume); bottom.Add(row.volume); bottom.Add(row.amount);
            root.Add(bottom);

            row.root = root;
            rows.Add(row);
            return root;
        }

        static string Words(string id) { return string.IsNullOrEmpty(id) ? "" : id.Replace('_', ' ').Replace('-', ' '); }

        static string Where(SatieSourceEntry e)
        {
            string place = e.atListener ? "at the listener" : !string.IsNullOrEmpty(e.host) ? "at " + Words(e.host) : e.spatial ? "placed" : "everywhere";
            return e.kind == "event" && !string.IsNullOrEmpty(e.@event) ? "on " + Words(e.@event) + ", " + place : place;
        }

        void FireFor(SatieSourceEntry entry, string surface)
        {
            if (core == null) return;
            if (!string.IsNullOrEmpty(entry.host)) SatieCore.HostEvent(entry.host, entry.@event, surface);
            else if (surface == null) SatieCore.Fire(entry.@event);
            else SatieCore.Fire(entry.@event, Vector3.zero, surface);
        }

        void Select(int source)
        {
            selected = source;
            if (manifest != null && core != null && source >= 0 && source < manifest.sources.Length)
            {
                Transform t = core.BoundTo(manifest.sources[source].host);
                if (t != null) { Selection.activeTransform = t; EditorGUIUtility.PingObject(t.gameObject); }
            }
            Refresh();
            SceneView.RepaintAll();
        }

        // ── Keeping the view current ──

        void Tick()
        {
            SatieCoreBehaviour now = EditorApplication.isPlaying ? SatieCoreBehaviour.Instance : null;
            if (now != core || (now != null && now.Loads != loadsSeen))
            {
                bool reloaded = now != null && now == core && now.Loads != loadsSeen;
                core = now;
                loadsSeen = now != null ? now.Loads : -1;
                if (reloaded) AfterReload();
                manifest = ReadManifest();
                Fill();
            }
            if (core != null && core.Session != null)
            {
                foreach (Row row in rows)
                {
                    float peak = core.TakeLevel(row.source);
                    float db = peak > 0f ? Mathf.Max(FloorDb, 20f * Mathf.Log10(peak)) : FloorDb;
                    row.shownDb = db > row.shownDb ? db : Mathf.Max(FloorDb, row.shownDb - 1.5f); // quick up, slow down
                    row.meter.style.width = Length.Percent(Mathf.Clamp01((row.shownDb - FloorDb) / -FloorDb) * 100f);
                }
                SatieStats stats = core.Stats;
                counters.text = stats.activeVoices + " playing    dropped " + stats.droppedVoices + "    rejected " + core.Session.Rejected;
                counters.style.color = stats.droppedVoices > 0 || core.Session.Rejected > 0 ? Warn : Dim;
                WatchFiles();
                SceneView.RepaintAll();
            }
        }

        void Refresh()
        {
            bool live = core != null && core.Session != null;
            if (manifest != null)
                status.text = !live ? "Press Play to hear and mix." : core.Session.CanMonitor ? "Playing. Changes are heard now and written on Save." : "Playing. This native library is older than the mixer: levels and volume are off.";
            if (!live) counters.text = "";
            // Only the test scene (SatieCoreDemo) can swap the output; a game sets it on its own component.
            SatieCoreDemo demo = live ? SatieCoreDemo.Instance : null;
            direction.style.display = demo != null ? DisplayStyle.Flex : DisplayStyle.None;
            if (demo != null) direction.text = demo.DirectionByUnity ? "Direction by Unity" : "Direction by Satie's stereo mix";
            foreach (Row row in rows)
            {
                bool muted = live && core.Muted(row.source), soloed = live && core.Soloed(row.source);
                float trim = live ? core.TrimDb(row.source) : 0f;
                row.volume.SetValueWithoutNotify(trim);
                row.volume.SetEnabled(live && core.Session.CanMonitor);
                row.mute.SetEnabled(live); row.solo.SetEnabled(live);
                row.amount.text = trim == 0f ? "as written" : (trim > 0f ? "+" : "") + trim.ToString("0", CultureInfo.InvariantCulture) + " dB";
                row.amount.style.color = trim == 0f ? Dim : Rose;
                row.mute.style.backgroundColor = muted ? Ink : Faint; row.mute.style.color = muted ? Color.black : Ink;
                row.solo.style.backgroundColor = soloed ? Rose : Faint; row.solo.style.color = soloed ? Color.black : Ink;
                row.root.style.opacity = muted ? 0.5f : 1f;
                row.root.style.backgroundColor = row.source == selected ? Faint : Color.clear;
                row.root.style.borderLeftColor = row.source == selected ? Rose : Color.clear;
                if (!live) { row.meter.style.width = Length.Percent(0); row.shownDb = FloorDb; }
            }
            foreach (KeyValuePair<string, Slider> pair in signalSliders)
            {
                pair.Value.SetEnabled(live);
                if (live) pair.Value.SetValueWithoutNotify(core.SignalValue(pair.Key));
            }
            string state = live ? core.CurrentState ?? "" : "";
            foreach (KeyValuePair<string, Button> pair in stateButtons)
            {
                bool on = live && pair.Key == state;
                pair.Value.SetEnabled(live);
                pair.Value.style.backgroundColor = on ? Rose : Faint; pair.Value.style.color = on ? Color.black : Ink;
            }
            int count = Pending().Count;
            changes.text = count == 0 ? "No unsaved changes." : count + (count == 1 ? " change" : " changes") + " not saved.";
            save.SetEnabled(count > 0);
            discard.SetEnabled(count > 0);
        }

        // ── Decisions: what is heard now against what Save has written ──

        struct Edit { public string voice, host, op, value; }

        List<Edit> Pending()
        {
            List<Edit> edits = new List<Edit>();
            if (core == null || core.Session == null || manifest == null) return edits;
            for (int i = 0; i < manifest.sources.Length; i++)
            {
                SatieSourceEntry s = manifest.sources[i];
                float trim = core.TrimDb(i), wasTrim;
                bool muted = core.Muted(i), wasMuted;
                savedTrim.TryGetValue(s.id, out wasTrim);
                savedMute.TryGetValue(s.id, out wasMuted);
                // The step since the last Save: satie_apply_edits adds each level line to the level it finds,
                // and this window cannot name the absolute level (spl) that lets a reader skip a line already applied.
                if (trim != wasTrim) edits.Add(new Edit { voice = s.id, host = s.host, op = "level", value = "{\"gainDb\":" + (trim - wasTrim).ToString("0.#", CultureInfo.InvariantCulture) + "}" });
                if (muted != wasMuted) edits.Add(new Edit { voice = s.id, host = s.host, op = muted ? "mute" : "unmute", value = "{}" });
            }
            foreach (KeyValuePair<string, Vector3> move in movedHosts)
                foreach (SatieHostEntry h in manifest.hostList)
                {
                    if (h.id != move.Key) continue;
                    foreach (int source in h.sources)
                        edits.Add(new Edit { voice = manifest.sources[source].id, host = h.id, op = "move", value = "{\"position\":[" + F(move.Value.x) + "," + F(move.Value.y) + "," + F(move.Value.z) + "]}" });
                }
            return edits;
        }

        static string F(float v) { return v.ToString("0.###", CultureInfo.InvariantCulture); }
        static string Quote(string s) { return s == null ? "null" : "\"" + s.Replace("\\", "\\\\").Replace("\"", "\\\"") + "\""; }

        public static string LogPath() { return Path.Combine(Path.GetDirectoryName(Application.dataPath), ".satie", "authoring", "edits.jsonl"); }

        void Save()
        {
            List<Edit> edits = Pending();
            if (edits.Count == 0) return;
            string at = DateTime.UtcNow.ToString("yyyy-MM-ddTHH:mm:ss.fffZ", CultureInfo.InvariantCulture);
            StringBuilder lines = new StringBuilder();
            foreach (Edit e in edits)
                lines.Append("{\"v\":\"satie.edits/v1\",\"at\":\"").Append(at).Append("\",\"scene\":").Append(Quote(manifest.name))
                    .Append(",\"voice\":").Append(Quote(e.voice)).Append(",\"host\":").Append(Quote(string.IsNullOrEmpty(e.host) ? null : e.host))
                    .Append(",\"op\":\"").Append(e.op).Append("\",\"value\":").Append(e.value).Append(",\"by\":\"developer\"}\n");
            string path = LogPath();
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            File.AppendAllText(path, lines.ToString(), new UTF8Encoding(false));
            for (int i = 0; i < manifest.sources.Length; i++)
            {
                savedTrim[manifest.sources[i].id] = core.TrimDb(i);
                savedMute[manifest.sources[i].id] = core.Muted(i);
            }
            movedHosts.Clear();
            Refresh();
            changes.text = edits.Count + (edits.Count == 1 ? " change" : " changes") + " written to .satie/authoring/edits.jsonl. Ask your coding agent to apply them.";
        }

        void Discard()
        {
            if (core == null || manifest == null) return;
            for (int i = 0; i < manifest.sources.Length; i++)
            {
                float trim; bool muted;
                savedTrim.TryGetValue(manifest.sources[i].id, out trim);
                savedMute.TryGetValue(manifest.sources[i].id, out muted);
                core.SetTrimDb(i, trim);
                core.SetMuted(i, muted);
            }
            foreach (KeyValuePair<string, Vector3> move in movedHosts)
                foreach (SatieHostEntry h in manifest.hostList)
                    if (h.id == move.Key && h.hasPosition) core.Session.MoveHost(h.id, h.position[0], h.position[1], h.position[2]);
            movedHosts.Clear();
            Refresh();
        }

        // A newly compiled scene has the saved decisions in it: the audition starts again from "as written".
        void AfterReload()
        {
            if (core == null || core.Session == null) return;
            for (int i = 0; i < core.Session.Manifest.sources.Length; i++) { core.SetTrimDb(i, 0f); core.SetMuted(i, false); }
            savedTrim.Clear();
            savedMute.Clear();
            movedHosts.Clear();
        }

        // ── Reload while playing: the compiled scene changed on disk ──

        void WatchFiles()
        {
            if (EditorApplication.timeSinceStartup < nextFileCheck) return;
            nextFileCheck = EditorApplication.timeSinceStartup + 1.0;
            if (!File.Exists(BundlePath) || !File.Exists(ManifestPath)) return;
            DateTime b = File.GetLastWriteTimeUtc(BundlePath), m = File.GetLastWriteTimeUtc(ManifestPath);
            if (bundleStamp == default(DateTime)) { bundleStamp = b; manifestStamp = m; return; }
            if (b == bundleStamp && m == manifestStamp) return;
            // Both files of a compile are written together; wait until neither is still being written.
            if ((DateTime.UtcNow - b).TotalSeconds < 1.0 || (DateTime.UtcNow - m).TotalSeconds < 1.0) return;
            bundleStamp = b; manifestStamp = m;
            // Straight from disk: the pair is checked against its own hash, so a torn write is refused, not played.
            if (core.Reload(File.ReadAllBytes(BundlePath), File.ReadAllText(ManifestPath)))
                Debug.Log("Satie: the scene changed on disk and was reloaded while playing.");
            AssetDatabase.ImportAsset(BundlePath);
            AssetDatabase.ImportAsset(ManifestPath);
        }

        // ── The Scene view: a dot where each sound is ──

        void OnScene(SceneView view)
        {
            if (manifest == null) return;
            bool live = core != null && core.Session != null;
            foreach (SatieHostEntry h in manifest.hostList)
            {
                Vector3 at;
                Transform bound = live ? core.BoundTo(h.id) : null;
                float x, y, z;
                if (bound != null) at = bound.position;
                else if (live && core.Session.HostPosition(h.id, out x, out y, out z)) at = new Vector3(x, y, z);
                else if (h.hasPosition) at = new Vector3(h.position[0], h.position[1], h.position[2]);
                else continue;

                bool chosen = false;
                float loud = FloorDb;
                int first = h.sources.Length > 0 ? h.sources[0] : 0;
                foreach (int source in h.sources)
                {
                    chosen |= source == selected;
                    foreach (Row row in rows) if (row.source == source) loud = Mathf.Max(loud, row.shownDb);
                }
                Color color = VoiceColors[first % VoiceColors.Length];
                float size = HandleUtility.GetHandleSize(at);
                float swell = Mathf.Clamp01((loud - FloorDb) / -FloorDb);
                Handles.color = new Color(color.r, color.g, color.b, 0.18f + 0.3f * swell);
                Handles.SphereHandleCap(0, at, Quaternion.identity, size * (0.14f + 0.3f * swell), EventType.Repaint);
                Handles.color = chosen ? Color.white : color;
                if (Handles.Button(at, Quaternion.identity, size * 0.07f, size * 0.12f, Handles.DotHandleCap)) Select(first);
                GUIStyle label = new GUIStyle(EditorStyles.miniLabel);
                label.normal.textColor = chosen ? Color.white : color;
                Handles.Label(at + Vector3.up * size * 0.22f, Words(h.id), label);

                // A place no game object holds can be moved here; a bound one moves with its object.
                if (live && chosen && bound == null)
                {
                    EditorGUI.BeginChangeCheck();
                    Vector3 next = Handles.PositionHandle(at, Quaternion.identity);
                    if (EditorGUI.EndChangeCheck())
                    {
                        core.Session.MoveHost(h.id, next.x, next.y, next.z);
                        movedHosts[h.id] = next;
                        Refresh();
                    }
                }
            }
        }
    }
}
