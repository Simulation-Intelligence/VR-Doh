using System.Collections;
using UnityEngine;
using UnityEngine.Rendering;

namespace MoonRabbit.World
{
    /// <summary>
    /// Intro (about 13 s, skippable with trigger / A / Space): the view fades in from black to the starry sky while
    /// faint stars drift upward around the player (a sense of coming down into the garden, no camera motion), and
    /// four story lines fade in and out one after another, world-fixed in front of the head. The Earth stays dark.
    /// Strikes only count once the intro is over (MoonProgress.IntroActive). Plays on start and after every reset.
    /// </summary>
    [RequireComponent(typeof(MoonStory))]
    public class StoryIntro : MonoBehaviour
    {
        public bool playIntro = true;
        [TextArea]
        public string[] lines =
        {
            "It's Chuseok on Earth… but the Moon never rose.",
            "No wishes. No mochi. No festival.",
            "In the sky garden between Earth and Moon, the Moon Rabbits pick up the mallet.",
            "!Pound the mochi. Light the Moon.",
        };
        [Tooltip("Reading time per line once it has fully risen (s); the instruction line gets a little more.")]
        public float holdTime = 2.2f;
        public float fadeTime = 0.6f;
        [Tooltip("Black at the start, fading to the scene over this many seconds.")]
        public float blackFade = 2.5f;
        [Tooltip("Lines longer than this many characters wrap onto two lines.")]
        public int wrapAt = 44;
        public Material fadeMaterial;
        [Header("Text styles")]
        [Tooltip("Story lines: Cinzel SemiBold, soft warm white, a little extra letter and line spacing (Cinzel's capitals " +
                 "and small capitals look alike, spacing reads better in VR).")]
        public StoryTextStyle narration = new StoryTextStyle
        {
            color = new Color(0.97f, 0.95f, 0.9f), letterHeight = 0.1f, characterSpacing = 6f, lineSpacing = 28f,
            glowIntensity = 0.16f, rise = true,
        };
        [Tooltip("Lines that tell the player what to do (start a line with '!'): Cinzel Black, moon gold, 1.2x, one pulse.")]
        public StoryTextStyle instruction = new StoryTextStyle
        {
            color = new Color(1f, 0.824f, 0.478f), letterHeight = 0.12f, characterSpacing = 5f, lineSpacing = 20f,
            glowColor = new Color(1f, 0.78f, 0.38f), glowIntensity = 0.24f, rise = true, pulse = true,
        };
        [Tooltip("Longest time a line takes to rise in letter by letter (s); short lines rise letter by letter more slowly.")]
        public float maxRevealTime = 1.6f;
        [Range(0, 300)] public int descentStars = 140;
        public Color descentColor = new Color(0.8f, 0.86f, 1f);

        public bool Playing { get; private set; }

        MoonStory story;
        MoonProgress progress;
        StoryText text, narrationText, instructionText;
        StorySounds sounds;
        GameObject fade;
        Renderer fadeRenderer;
        MaterialPropertyBlock mpb;
        Material starHalo;
        Matrix4x4[] starM;
        Vector3[] starPos;
        float starAlpha;
        bool skip;
        float textAlpha;
        Coroutine running;

        void Awake()
        {
            story = GetComponent<MoonStory>();
            progress = GetComponent<MoonProgress>();
            sounds = GetComponent<StorySounds>();
            mpb = new MaterialPropertyBlock();
        }

        void Start()
        {
            progress.SkipIntroRequested += () => skip = true;
            story.Resetted += () => Play(progress.LastResetReason);
            starHalo = story.HaloMaterialFor("Descent", descentColor);
            Play("launch");
        }

        void OnDestroy()
        {
            if (starHalo != null) Destroy(starHalo);
        }

        int lastPlayFrame = -1;

        public void Play() => Play("manual");

        /// <summary>(Re)starts the intro once: a running intro is stopped and its text cleared first.</summary>
        public void Play(string reason)
        {
            if (!playIntro || !isActiveAndEnabled) return;
            if (Time.frameCount == lastPlayFrame) return; // two triggers in one frame start it only once
            lastPlayFrame = Time.frameCount;
            if (running != null) { StopCoroutine(running); running = null; }
            if (sounds != null) sounds.StopTextSounds();
            // a restarted intro never leaves the previous line on screen
            if (narrationText != null) narrationText.SetAlpha(0f);
            if (instructionText != null) instructionText.SetAlpha(0f);
            Playing = false;
            Debug.Log("[Moon Rabbit] Intro start (reason: " + reason + ")");
            running = StartCoroutine(Intro());
        }

        IEnumerator Intro()
        {
            var cam = MoonStory.MainCamera();
            while (cam == null) { yield return null; cam = MoonStory.MainCamera(); }
            Playing = true;
            introStart = Time.time;
            skip = false;
            progress.IntroActive = true;
            story.EarthBrightness = 0f;
            EnsureFade(cam.transform);
            SetFade(1f);
            SpawnStars(cam.transform);
            if (narrationText == null) narrationText = StoryText.Create(transform, "MR_IntroText", narration, story.TextHalo, story.quadMesh);
            text = narrationText;
            // world-fixed: placed once in front of the head, facing the Moon side of the garden
            yield return null; // let the head pose settle for a frame
            narrationText.PlaceInFront(cam.transform, story.textDistance, story.textAboveEyes);

            introStart = Time.time;
            for (int idx = 0; idx < lines.Length && !skip; idx++)
            {
                // a second of stars before the first line
                while (!skip && Time.time - introStart < 1f) yield return null;
                if (skip) break;

                // one text object for every line: "[...]" (or a whole line starting with "!") is the instruction emphasis
                string raw = lines[idx].StartsWith("!") ? "[" + lines[idx].Substring(1) + "]" : lines[idx];
                bool isInstruction = raw.Contains("[");
                text = narrationText;
                text.maxReveal = maxRevealTime;
                string rich = Emphasise(raw, out int from);
                text.SetText(rich, from);
                // a soft chime from the text, so it is heard (and located) even when looking elsewhere;
                // the instruction line rings at its bounce
                if (sounds != null) sounds.PlayIntroLine(text.transform.position, idx, isInstruction, isInstruction ? text.RevealDuration : 0.1f);
                text.SetAlpha(1f); // letters fade in themselves as they rise
                progress.onIntroLine.Invoke(idx + 1);

                // reading time counts from when the whole line has risen
                float hold = text.RevealDuration + (isInstruction ? holdTime + 0.6f : holdTime);
                float t = 0f;
                while (!skip && t < hold) { t += Time.deltaTime; yield return null; }
                if (skip) break;

                text.BeginExit();
                t = 0f;
                float exit = text.ExitDuration;
                while (!skip && t < exit) { t += Time.deltaTime; yield return null; }
                text.SetAlpha(0f);
            }

            // end (or skipped): clear text at once, fade out whatever else is showing
            if (skip && sounds != null) sounds.StopTextSounds();
            if (narrationText != null) narrationText.SetAlpha(0f);
            if (instructionText != null) instructionText.SetAlpha(0f);
            float a0 = fadeRenderer != null ? CurrentFade : 0f;
            for (float a = 0f; a < 0.35f; a += Time.deltaTime)
            {
                float k = 1f - a / 0.35f;
                SetFade(a0 * k);
                starAlpha *= k;
                yield return null;
            }
            SetFade(0f);
            starAlpha = 0f;
            progress.IntroActive = false;
            Playing = false;
            running = null;
        }

        float introStart = -100f;

        void LateUpdate()
        {
            // black fade-in and the drifting stars run on their own clock while the lines play
            if (!Playing || skip) return;
            float t = Time.time - introStart;
            SetFade(1f - Mathf.SmoothStep(0f, 1f, t / blackFade));
            starAlpha = Mathf.Clamp01(t / 1.5f) * Mathf.Clamp01((7f - t) / 2f);
        }

        /// <summary>Wraps the plain line and turns "[...]" into the instruction look (gold, bold = Cinzel Black via the
        /// font's bold weight, a little larger). <paramref name="from"/> = index of the first emphasised visible character.</summary>
        string Emphasise(string raw, out int from)
        {
            int open = raw.IndexOf('['), close = raw.IndexOf(']');
            string plain = raw.Replace("[", "").Replace("]", "");
            string wrapped = Wrap(plain);
            from = -1;
            if (open < 0 || close < open) return wrapped;
            int a = open, b = close - 1; // plain indices (the brackets are gone); Wrap only swaps a space for a newline
            from = a;
            float size = narration.letterHeight > 0f ? instruction.letterHeight / narration.letterHeight * 100f : 120f;
            string col = ColorUtility.ToHtmlStringRGB(instruction.color);
            return wrapped.Substring(0, a) + "<b><color=#" + col + "><size=" + size.ToString("0") + "%>" +
                   wrapped.Substring(a, b - a) + "</size></color></b>" + wrapped.Substring(b);
        }

        string Wrap(string s)
        {
            if (s.Length <= wrapAt) return s;
            int mid = s.Length / 2, cut = -1;
            for (int d = 0; d < mid; d++)
            {
                if (mid + d < s.Length && s[mid + d] == ' ') { cut = mid + d; break; }
                if (mid - d > 0 && s[mid - d] == ' ') { cut = mid - d; break; }
            }
            return cut < 0 ? s : s.Substring(0, cut) + "\n" + s.Substring(cut + 1);
        }

        // ---- fade sphere (head-locked; only alpha changes)

        float CurrentFade { get; set; }

        void EnsureFade(Transform head)
        {
            if (fade != null || fadeMaterial == null || story.sphereMesh == null) return;
            fade = new GameObject("MR_IntroFade");
            fade.transform.SetParent(head, false);
            fade.transform.localScale = Vector3.one * 0.6f;
            fade.AddComponent<MeshFilter>().sharedMesh = story.sphereMesh;
            fadeRenderer = fade.AddComponent<MeshRenderer>();
            fadeRenderer.sharedMaterial = fadeMaterial;
            fadeRenderer.shadowCastingMode = ShadowCastingMode.Off;
            fadeRenderer.receiveShadows = false;
        }

        void SetFade(float a)
        {
            CurrentFade = a;
            if (fadeRenderer == null) return;
            fadeRenderer.enabled = a > 0.002f;
            fadeRenderer.GetPropertyBlock(mpb);
            mpb.SetColor("_Color", new Color(0f, 0f, 0f, a));
            fadeRenderer.SetPropertyBlock(mpb);
        }

        // ---- stars drifting upward (we seem to descend), world-fixed around where the player stands

        void SpawnStars(Transform head)
        {
            if (descentStars <= 0) return;
            starM = new Matrix4x4[descentStars];
            starPos = new Vector3[descentStars];
            var rng = new System.Random(5);
            var c = head.position;
            for (int i = 0; i < descentStars; i++)
            {
                float a = (float)rng.NextDouble() * Mathf.PI * 2f;
                float r = Mathf.Lerp(2.5f, 9f, (float)rng.NextDouble());
                float y = Mathf.Lerp(-4f, 7f, (float)rng.NextDouble());
                starPos[i] = c + new Vector3(Mathf.Cos(a) * r, y, Mathf.Sin(a) * r);
            }
        }

        void Update()
        {
            if (starAlpha <= 0.001f || starPos == null || starHalo == null) return;
            var cam = MoonStory.MainCamera();
            if (cam == null) return;
            var head = cam.transform.position;
            for (int i = 0; i < starPos.Length; i++)
            {
                starPos[i] += Vector3.up * (0.45f * Time.deltaTime);       // slow: a hint of descent, not motion
                if (starPos[i].y > head.y + 7f) starPos[i].y -= 11f;
                float edge = Mathf.Clamp01((head.y + 7f - starPos[i].y) / 2f) * Mathf.Clamp01((starPos[i].y - head.y + 4f) / 2f);
                float d = Vector3.Distance(head, starPos[i]);
                float tw = 0.7f + 0.3f * Mathf.Sin(Time.time * 2.3f + i * 1.7f);
                starM[i] = Matrix4x4.TRS(starPos[i], Quaternion.LookRotation(starPos[i] - head), Vector3.one * d * 0.012f * tw * edge);
            }
            story.DrawHalos(starHalo, starM, starM.Length, 0.6f * starAlpha);
        }
    }
}
