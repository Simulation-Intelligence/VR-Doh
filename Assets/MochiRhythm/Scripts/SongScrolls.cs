using System.Collections.Generic;
using MoonRabbit.World;
using TMPro;
using UnityEngine;

namespace MochiRhythm
{
    /// <summary>
    /// Song choice without buttons. When the story is idle (intro over, no ending playing) and no song runs, one small
    /// hanji scroll per song unrolls above the usu (Jihyeon's IntroScroll), titled with the song and, underneath, the
    /// mallet it holds. Grabbing a scroll (pinch, closed hand or grip) turns it into that mallet in the hand and starts the
    /// song; the other scrolls roll up. When the song ends (full Moon, see <see cref="RhythmStoryBridge"/>) the mallet is
    /// let go, shrinks away and the scrolls come back once the story has reset.
    /// Until a scroll is picked the kine is hidden and cannot be grabbed. The button panel is hidden.
    /// </summary>
    public class SongScrolls : MonoBehaviour
    {
        public RhythmMode mode;
        [Header("Layout (station space: the kneader stands at -Z)")]
        public Vector2 paperSize = new Vector2(0.22f, 0.1f);
        public float spacing = 0.29f;
        public float aboveRim = 0.3f;
        public float beyondAxis = 0.04f;
        [Tooltip("Song title size on the paper (TextMeshPro points, ~0.1 m each); long titles shrink to fit.")]
        public float titleSize = 0.3f;
        [Header("Timing (s)")]
        public float openTime = 0.7f, closeTime = 0.6f, morphTime = 0.35f, returnTime = 0.45f;
        [Tooltip("Story idle this long before the scrolls unroll.")]
        public float offerDelay = 0.8f;

        class Slot
        {
            public int song;
            public Mallet mallet;
            public IntroScroll scroll;
            public GrabZone zone;
            public TextMeshPro title;
            public float openedAt, glow;
        }

        enum State { Hidden, Offering, Holding, Returning }
        State state = State.Hidden;
        readonly List<Slot> slots = new List<Slot>();
        List<Mallet> mallets;
        Mallet yokogine, current;
        StoryIntro intro;
        MoonStory story;
        StorySounds sounds;
        KineTool kine;
        KineContact contact;
        GrabZone kineZone;
        MeshFilter skin;
        MeshRenderer skinRenderer;
        Mesh baseSkinMesh;
        Material[] baseSkinMaterials, malletMaterials;
        readonly List<Renderer> kineRenderers = new List<Renderer>();
        readonly List<bool> kineRendererWasOn = new List<bool>();
        readonly Dictionary<Mallet, Mesh> meshes = new Dictionary<Mallet, Mesh>();
        float idleSince = -1f, stateSince;
        bool songStarted;
        Transform pose;

        void Start()
        {
            if (mode == null || mode.Kine == null || mode.KineZone == null || mode.Grabber == null)
            {
                Debug.LogWarning("[rhythm] song scrolls: no kine, grab zone or grabber; keeping the button panel");
                enabled = false;
                return;
            }
            intro = FindAnyObjectByType<StoryIntro>();
            story = FindAnyObjectByType<MoonStory>();
            sounds = FindAnyObjectByType<StorySounds>();
            if (intro == null || story == null || intro.scrollMaterial == null || story.quadMesh == null)
            {
                Debug.LogWarning("[rhythm] song scrolls: the Moon story's scroll is missing; keeping the button panel");
                enabled = false;
                return;
            }
            if (mode.Panel != null) mode.Panel.gameObject.SetActive(false);

            kine = mode.Kine;
            kineZone = mode.KineZone;
            contact = kine.GetComponent<KineContact>();
            foreach (var r in kine.GetComponentsInChildren<Renderer>(true)) { kineRenderers.Add(r); kineRendererWasOn.Add(r.enabled); }
            foreach (var mf in kine.GetComponentsInChildren<MeshFilter>(true))
                if (mf.name == "KineSkin") { skin = mf; skinRenderer = mf.GetComponent<MeshRenderer>(); }
            if (skin != null) { baseSkinMesh = skin.sharedMesh; baseSkinMaterials = skinRenderer.sharedMaterials; }
            mallets = Mallet.All(kine, kineZone, contact != null ? contact.handleRadius : 0.0175f);
            yokogine = mallets.Find(m => m.build == null);
            malletMaterials = FindMalletMaterials();

            pose = new GameObject("Scroll pose").transform;
            pose.SetParent(transform, false);
            BuildSlots();
            HideKine();
            mode.Driver.Rest();
            Debug.Log($"[rhythm] song scrolls: {slots.Count} songs -> " + string.Join(", ", slots.ConvertAll(s => mode.Music.NameOf(s.song) + " = " + s.mallet.name)));
        }

        // ---- songs and their mallets ----

        void BuildSlots()
        {
            var m = mode.Music;
            var used = new HashSet<Mallet>();
            for (int i = 0; i < m.SongCount; i++)
            {
                string n = m.NameOf(i).ToLowerInvariant();
                var mallet = mallets.Find(x => !used.Contains(x) && n.Contains(x.songMatch));
                if (mallet != null) used.Add(mallet);
                slots.Add(new Slot { song = i, mallet = mallet });
            }
            // songs without a named mallet take the ones left, then share them in turn
            int next = 0;
            foreach (var s in slots)
            {
                if (s.mallet != null) continue;
                var free = mallets.Find(x => !used.Contains(x));
                if (free != null) { used.Add(free); s.mallet = free; }
                else s.mallet = mallets[next++ % mallets.Count];
            }
            // heaviest mallet on the left, as the list is laid out (tteokme, yokogine, pestle)
            slots.Sort((a, b) => mallets.IndexOf(a.mallet).CompareTo(mallets.IndexOf(b.mallet)));

            foreach (var s in slots)
            {
                s.scroll = IntroScroll.Create(transform, intro.scrollMaterial, story.quadMesh, intro.narration.font, intro.hintMaterial);
                s.scroll.name = "Song scroll " + Pretty(mode.Music.NameOf(s.song));
                s.scroll.padding = new Vector2(0.025f, 0.02f);
                s.scroll.margin = 0.03f;
                s.scroll.hintText = s.mallet.Label;
                s.scroll.hintDelay = 0.35f;

                var t = new GameObject("Title");
                t.transform.SetParent(s.scroll.transform, false);
                s.title = t.AddComponent<TextMeshPro>();
                if (intro.narration.font is TMP_FontAsset fa) s.title.font = fa;
                if (intro.narration.material != null) s.title.fontSharedMaterial = intro.narration.material;
                s.title.text = Pretty(mode.Music.NameOf(s.song));
                s.title.alignment = TextAlignmentOptions.Center;
                s.title.textWrappingMode = TextWrappingModes.NoWrap;
                s.title.enableAutoSizing = false;
                s.title.rectTransform.sizeDelta = paperSize;
                // 3D TextMeshPro: ~0.1 m per point. Letters about 3 cm, smaller if the title is wider than the paper.
                s.title.fontSize = titleSize;
                s.title.ForceMeshUpdate(true);
                float wide = s.title.preferredWidth;
                if (wide > paperSize.x * 0.92f) s.title.fontSize = titleSize * paperSize.x * 0.92f / wide;
                s.title.color = Clear(intro.narration.color);
                var tr = t.GetComponent<MeshRenderer>();
                tr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                tr.receiveShadows = false;

                s.zone = s.scroll.gameObject.AddComponent<GrabZone>();
                s.zone.shape = GrabZone.Shape.Segment;
                s.zone.moveTarget = s.scroll.transform;
                s.zone.segmentA = new Vector3(-paperSize.x * 0.5f, 0f, 0f);
                s.zone.segmentB = new Vector3(paperSize.x * 0.5f, 0f, 0f);
                s.zone.segmentRadius = paperSize.y * 0.6f;
                s.zone.highlight = new Renderer[] { tr }; // the hover tint must not touch the paper's property block
                var slot = s;
                s.zone.Grabbed += _ => Pick(slot);
                mode.Grabber.zones.Add(s.zone);
            }
        }

        /// <summary>"Skrillex_-_Bangarang_feat._Sirah_(mp3.pm)" -> "Bangarang"; "PSY - Gangnam Style" -> "Gangnam Style".</summary>
        static string Pretty(string file)
        {
            string s = file.Replace('_', ' ');
            int p = s.IndexOf('('); if (p > 0) s = s.Substring(0, p);
            int d = s.IndexOf(" - "); if (d >= 0) s = s.Substring(d + 3);
            int f = s.IndexOf(" feat"); if (f > 0) s = s.Substring(0, f);
            int r = s.IndexOf(" rus "); if (r > 0) s = s.Substring(0, r);
            return s.Trim();
        }

        static Color Clear(Color c) { c.a = 0f; return c; }

        Material[] FindMalletMaterials()
        {
            var want = new[] { "MR_KineHandle", "MR_KineHead", "MR_SkinRabbitPink", "MR_UsuWood", "MR_UsuEndGrain" };
            var found = new Material[want.Length];
            foreach (var r in FindObjectsByType<Renderer>(FindObjectsInactive.Include, FindObjectsSortMode.None))
                foreach (var m in r.sharedMaterials)
                    if (m != null) { int i = System.Array.IndexOf(want, m.name); if (i >= 0 && found[i] == null) found[i] = m; }
            // fall back to the kine skin's own wood and pink
            var fallback = baseSkinMaterials != null && baseSkinMaterials.Length > 0 ? baseSkinMaterials[0] : null;
            for (int i = 0; i < found.Length; i++)
                if (found[i] == null) { found[i] = i == 4 && found[1] != null ? found[1] : i == 3 && found[0] != null ? found[0] : fallback; Debug.LogWarning("[rhythm] song scrolls: no " + want[i] + " in the scene, using a stand-in"); }
            return found;
        }

        // ---- states ----

        bool StoryIdle()
        {
            var mp = MoonProgress.Instance;
            if (intro != null && intro.Playing) return false;
            return mp == null || (!mp.IntroActive && !mp.Locked && !mp.IsComplete);
        }

        void Update()
        {
            var g = mode.Game;
            var m = mode.Music;
            switch (state)
            {
                case State.Hidden:
                    bool ready = StoryIdle() && !g.Active && !m.IsLoading && !m.IsPlaying && !mode.Suspended;
                    if (!ready) { idleSince = -1f; break; }
                    if (idleSince < 0f) idleSince = Time.time;
                    if (Time.time - idleSince >= offerDelay) Offer();
                    break;
                case State.Offering:
                    if (!StoryIdle()) { CloseAll(null); state = State.Hidden; }
                    break;
                case State.Holding:
                    if (g.Active) songStarted = true;
                    bool loadFailed = !songStarted && !m.IsLoading && Time.time - stateSince > 2f && !g.Active;
                    if ((songStarted && !g.Active) || loadFailed)
                    {
                        if (loadFailed) Debug.LogWarning("[rhythm] song scrolls: the song did not start (" + m.Status + ")");
                        BeginReturn();
                    }
                    break;
                case State.Returning:
                    float k = Mathf.Clamp01((Time.time - stateSince) / returnTime);
                    SetSkinScale(1f - k * k);
                    if (k >= 1f) FinishReturn();
                    break;
            }
            UpdateScrolls();
            if (state == State.Holding) SetSkinScale(EaseOutBack(Mathf.Clamp01((Time.time - stateSince) / morphTime)));
        }

        void Offer()
        {
            var st = mode.Station.transform;
            var cam = Camera.main;
            Vector3 head = cam != null ? cam.transform.position : st.TransformPoint(new Vector3(0f, 0.5f, -0.6f));
            Vector3 centre = mode.RimCenter + Vector3.up * aboveRim + st.forward * beyondAxis;
            for (int i = 0; i < slots.Count; i++)
            {
                var s = slots[i];
                Vector3 p = centre + st.right * ((i - (slots.Count - 1) * 0.5f) * spacing);
                Vector3 away = p - head; away.y = 0f;
                if (away.sqrMagnitude < 1e-4f) away = st.forward;
                pose.SetPositionAndRotation(p, Quaternion.LookRotation(away.normalized, Vector3.up)); // text readable from -Z
                s.scroll.Setup(pose, paperSize, 0.22f);
                s.scroll.Open(openTime);
                s.scroll.CloudGlow = 0.2f;
                s.openedAt = Time.time;
                s.zone.enabled = true;
            }
            if (sounds != null && slots.Count > 0) sounds.PlayScrollUnroll(centre, st.right, openTime);
            state = State.Offering;
            stateSince = Time.time;
        }

        void CloseAll(Slot except)
        {
            foreach (var s in slots)
            {
                s.zone.enabled = false;
                if (s == except || !s.scroll.gameObject.activeSelf) continue;
                s.scroll.Close(closeTime);
                s.openedAt = -1f;
            }
            if (sounds != null) sounds.PlayScrollClose(mode.RimCenter + Vector3.up * aboveRim, closeTime);
        }

        void UpdateScrolls()
        {
            foreach (var s in slots)
            {
                if (s.scroll == null || !s.scroll.gameObject.activeSelf) continue;
                bool open = s.openedAt >= 0f;
                float a = open ? Mathf.Clamp01((Time.time - s.openedAt - openTime * 0.6f) / 0.4f) : Mathf.Max(0f, s.title.color.a - Time.deltaTime * 4f);
                var c = intro.narration.color; c.a = a;
                s.title.color = c;
                float want = s.zone.enabled && s.zone.Hovered ? 1f : 0f;
                s.glow = Mathf.MoveTowards(s.glow, want, Time.deltaTime * 5f);
                s.scroll.CloudGlow = 0.2f + 1.1f * s.glow;
                s.scroll.transform.localScale = Vector3.one * (1f + 0.07f * s.glow);
            }
        }

        // ---- scroll -> mallet ----

        void Pick(Slot s)
        {
            if (state != State.Offering) return;
            mode.Grabber.GripPose(s.zone, out Vector3 hand, out _);
            CloseAll(s);
            s.scroll.Fade(0.2f);
            s.openedAt = -1f;

            Apply(s.mallet);
            // the mallet appears in the hand: head down, handle toward the player, the hand on its grip
            Vector3 fwd = s.scroll.transform.forward; fwd.y = 0f;
            if (fwd.sqrMagnitude < 1e-4f) fwd = mode.Station.transform.forward;
            Quaternion rot = Quaternion.LookRotation(fwd.normalized, Vector3.up);
            kine.transform.SetPositionAndRotation(hand - rot * s.mallet.handPoint, rot);
            ShowKine();
            kineZone.enabled = true;
            if (!mode.Grabber.Handover(s.zone, kineZone)) Debug.LogWarning("[rhythm] song scrolls: nobody held the scroll");
            SetSkinScale(0.2f);
            if (sounds != null) sounds.PlayScrollUnroll(hand, fwd, morphTime);

            songStarted = false;
            state = State.Holding;
            stateSince = Time.time;
            int song = s.song;
            Debug.Log($"[rhythm] song scroll picked: {mode.Music.NameOf(song)} with the {s.mallet.name}");
            mode.Music.Select(song, ok =>
            {
                if (ok && state == State.Holding && mode.Music.Play()) mode.Game.StartGame(0);
            });
        }

        void Apply(Mallet m)
        {
            current = m;
            kine.headRadius = m.headRadius;
            kine.headLength = m.headLength;
            kine.faceRounding = m.faceRounding;
            kine.Rebuild();
            kineZone.segmentA = m.gripA;
            kineZone.segmentB = m.gripB;
            kineZone.segmentRadius = m.gripRadius;
            if (contact != null) { contact.handleRadius = m.handleRadius; contact.Rebuild(); }
            mode.Driver.handleRadius = m.handleRadius;
            mode.Driver.tiltOverRim = !m.vertical;
            if (skin == null) return;
            if (m.build == null) { skin.sharedMesh = baseSkinMesh; skinRenderer.sharedMaterials = baseSkinMaterials; }
            else
            {
                if (!meshes.TryGetValue(m, out var mesh)) meshes[m] = mesh = m.build(m);
                skin.sharedMesh = mesh;
                skinRenderer.sharedMaterials = malletMaterials;
            }
        }

        void BeginReturn()
        {
            mode.Grabber.ForceRelease(kineZone);
            kineZone.enabled = false;
            state = State.Returning;
            stateSince = Time.time;
        }

        void FinishReturn()
        {
            HideKine();
            if (yokogine != null) Apply(yokogine); // the team's own kine shape while nobody holds a mallet
            current = null;
            mode.Driver.Rest();
            SetSkinScale(1f);
            idleSince = -1f;
            state = State.Hidden;
        }

        void HideKine()
        {
            foreach (var r in kineRenderers) if (r != null) r.enabled = false;
            kineZone.enabled = false;
        }

        void ShowKine()
        {
            for (int i = 0; i < kineRenderers.Count; i++)
            {
                var r = kineRenderers[i];
                if (r == null) continue;
                // a mallet with its own mesh shows only the skin, not the team's plain cylinders
                r.enabled = current != null && current.build != null ? r == skinRenderer : kineRendererWasOn[i];
            }
        }

        void SetSkinScale(float s)
        {
            if (skin != null) skin.transform.localScale = Vector3.one * Mathf.Max(0.001f, s);
        }

        static float EaseOutBack(float t)
        {
            const float c = 1.6f;
            float u = t - 1f;
            return 1f + (c + 1f) * u * u * u + c * u * u;
        }

        void OnDestroy()
        {
            foreach (var mesh in meshes.Values) if (mesh != null) Destroy(mesh);
        }
    }
}
