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
            "No wishes. No rice cakes. No festival.",
            "In the sky garden between Earth and Moon, the Moon Rabbits pick up the mallet.",
            "Pound the mochi. Light the Moon.",
        };
        [Tooltip("Seconds per line (fade in, hold, fade out).")]
        public float lineTime = 3.2f;
        public float fadeTime = 0.6f;
        [Tooltip("Black at the start, fading to the scene over this many seconds.")]
        public float blackFade = 2.5f;
        [Tooltip("Lines longer than this many characters wrap onto two lines.")]
        public int wrapAt = 44;
        public Material fadeMaterial;
        [Range(0, 300)] public int descentStars = 140;
        public Color descentColor = new Color(0.8f, 0.86f, 1f);

        public bool Playing { get; private set; }

        MoonStory story;
        MoonProgress progress;
        StoryText text;
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
            mpb = new MaterialPropertyBlock();
        }

        void Start()
        {
            progress.SkipIntroRequested += () => skip = true;
            story.Resetted += Play;
            starHalo = story.HaloMaterialFor("Descent", descentColor);
            Play();
        }

        void OnDestroy()
        {
            if (starHalo != null) Destroy(starHalo);
        }

        public void Play()
        {
            if (!playIntro || !isActiveAndEnabled) return;
            if (running != null) StopCoroutine(running);
            running = StartCoroutine(Intro());
        }

        IEnumerator Intro()
        {
            var cam = MoonStory.MainCamera();
            while (cam == null) { yield return null; cam = MoonStory.MainCamera(); }
            Playing = true;
            skip = false;
            progress.IntroActive = true;
            story.EarthBrightness = 0f;
            EnsureFade(cam.transform);
            SetFade(1f);
            SpawnStars(cam.transform);
            if (text == null) text = StoryText.Create(transform, "MR_IntroText", story.textHeight, story.textColor, story.TextHalo, story.quadMesh, 0.16f);
            // world-fixed: placed once in front of the head, facing the Moon side of the garden
            yield return null; // let the head pose settle for a frame
            text.PlaceInFront(cam.transform, story.textDistance, story.textAboveEyes);

            float t0 = Time.time;
            int line = -1;
            textAlpha = 0f;
            while (!skip)
            {
                float t = Time.time - t0;
                SetFade(1f - Mathf.SmoothStep(0f, 1f, t / blackFade));
                starAlpha = Mathf.Clamp01(t / 1.5f) * Mathf.Clamp01((7f - t) / 2f);

                float lt = t - 1f; // first line after a second of stars
                int idx = lt < 0f ? -1 : Mathf.FloorToInt(lt / lineTime);
                if (idx >= lines.Length) break;
                if (idx >= 0)
                {
                    if (idx != line)
                    {
                        line = idx;
                        text.SetText(Wrap(lines[idx]));
                        progress.onIntroLine.Invoke(idx + 1);
                    }
                    float u = lt - idx * lineTime;
                    textAlpha = Mathf.Clamp01(u / fadeTime) * Mathf.Clamp01((lineTime - u) / fadeTime);
                    text.SetAlpha(textAlpha);
                }
                yield return null;
            }

            // end (or skipped): quick fade of whatever is still showing
            float a0 = fadeRenderer != null ? CurrentFade : 0f;
            for (float a = 0f; a < 0.35f; a += Time.deltaTime)
            {
                float k = 1f - a / 0.35f;
                SetFade(a0 * k);
                text.SetAlpha(textAlpha * k);
                starAlpha *= k;
                yield return null;
            }
            SetFade(0f);
            text.SetAlpha(0f);
            starAlpha = 0f;
            progress.IntroActive = false;
            Playing = false;
            running = null;
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
