using UnityEngine;
using UnityEngine.Rendering;
#if MR_TMP
using TMPro;
#endif

namespace MoonRabbit.World
{
    /// <summary>
    /// The hanji handscroll the intro lines are written on (MoonRabbit/HanjiScroll on one quad, plus a small "Skip" hint
    /// below it). It unrolls from the middle once at the start of the intro, the four lines rise and drift away on it,
    /// and it rolls up again after the last line (or fades at once on a skip). Its size is fixed for the whole intro
    /// (the longest line). The paper's fibre texture and the gold cloud motif are drawn in code once (512² and 256²).
    /// </summary>
    public class IntroScroll : MonoBehaviour
    {
        [Tooltip("Space between the longest line and the paper's edge (m).")]
        public Vector2 padding = new Vector2(0.17f, 0.1f);
        [Tooltip("Extra room on the quad around the paper for the rolls and torn edges (m).")]
        public float margin = 0.05f;
        public string hintText = "Skip"; // A, trigger, Space or a held pinch skips (MoonProgress)
        public Color hintColor = new Color(0.93f, 0.88f, 0.76f, 0.62f);
        [Tooltip("Hint fades in this long after the intro starts (s).")]
        public float hintDelay = 1f;

        Renderer paper;
        MaterialPropertyBlock mpb;
        Vector2 size;
        float unroll, alpha, from, to, t0, dur = 1f, hintStart = -100f, hintAlpha;
        bool hintOn;
        public float CloudGlow { get; set; }
        /// <summary>True while the paper is unrolling, rolling up or fading.</summary>
        public bool Moving => Time.time - t0 < dur;
#if MR_TMP
        TextMeshPro hint;
#endif

        static Texture2D fiberTex, cloudTex;

        public static IntroScroll Create(Transform parent, Material material, Mesh quad, Object hintFont, Material hintMaterial)
        {
            var go = new GameObject("MR_IntroScroll");
            go.transform.SetParent(parent, true);
            var sc = go.AddComponent<IntroScroll>();
            sc.mpb = new MaterialPropertyBlock();
            var p = new GameObject("Paper");
            p.transform.SetParent(go.transform, false);
            p.transform.localPosition = new Vector3(0f, 0f, 0.035f); // just behind the letters
            p.AddComponent<MeshFilter>().sharedMesh = quad;
            var r = p.AddComponent<MeshRenderer>();
            r.sharedMaterial = material;
            r.shadowCastingMode = ShadowCastingMode.Off;
            r.receiveShadows = false;
            r.lightProbeUsage = LightProbeUsage.Off;
            sc.paper = r;
            EnsureTextures();
            sc.mpb.SetTexture("_FiberTex", fiberTex);
            sc.mpb.SetTexture("_CloudTex", cloudTex);
#if MR_TMP
            var h = new GameObject("SkipHint");
            h.transform.SetParent(go.transform, false);
            sc.hint = h.AddComponent<TextMeshPro>();
            if (hintFont is TMP_FontAsset fa) sc.hint.font = fa;
            if (hintMaterial != null) sc.hint.fontSharedMaterial = hintMaterial;
            sc.hint.alignment = TextAlignmentOptions.Center;
            sc.hint.textWrappingMode = TextWrappingModes.NoWrap;
            sc.hint.characterSpacing = 8f;
            sc.hint.text = sc.hintText;
            sc.hint.rectTransform.sizeDelta = new Vector2(4f, 0.5f);
            var hr = h.GetComponent<MeshRenderer>();
            hr.shadowCastingMode = ShadowCastingMode.Off;
            hr.receiveShadows = false;
            h.SetActive(false);
#endif
            go.SetActive(false);
            return sc;
        }

        /// <summary>Places the scroll where the text is, sized for <paramref name="longestLine"/> (m), rolled up and hidden.</summary>
        public void Setup(Transform textPose, Vector2 longestLine, float hintFontSize)
        {
            transform.SetPositionAndRotation(textPose.position, textPose.rotation);
            size = longestLine + padding * 2f;
            paper.transform.localScale = new Vector3(size.x + margin * 2f, size.y + margin * 2f, 1f);
#if MR_TMP
            if (hint != null)
            {
                hint.text = hintText;
                hint.fontSize = hintFontSize;
                hint.transform.localPosition = new Vector3(0f, -(size.y * 0.5f + 0.075f), 0f);
                hint.gameObject.SetActive(false);
            }
#endif
            unroll = 0f; alpha = 0f; from = to = 0f; t0 = -100f; hintOn = false; hintAlpha = 0f; CloudGlow = 0f;
            gameObject.SetActive(true);
            Apply();
        }

        /// <summary>Unrolls from the middle to both ends over <paramref name="time"/> s; the skip hint comes a moment later.</summary>
        public void Open(float time)
        {
            alpha = 1f; Animate(unroll, 1f, time);
            hintStart = Time.time + hintDelay; hintOn = true;
        }

        /// <summary>Rolls up to the middle (both ends towards the centre), then hides.</summary>
        public void Close(float time) { HideHint(); Animate(unroll, 0f, time); }

        /// <summary>Fades out quickly where it is (on a skip).</summary>
        public void Fade(float time) { HideHint(); fadeFrom = alpha; fading = true; t0 = Time.time; dur = Mathf.Max(0.05f, time); }

        public void HideHint()
        {
            hintOn = false; hintAlpha = 0f;
#if MR_TMP
            if (hint != null) hint.gameObject.SetActive(false);
#endif
        }

        /// <summary>Gone at once (a new intro, or the component is reset).</summary>
        public void HideNow() { HideHint(); unroll = 0f; alpha = 0f; fading = false; t0 = -100f; gameObject.SetActive(false); }

        bool fading;
        float fadeFrom;

        void Animate(float a, float b, float time)
        {
            fading = false;
            from = a; to = b; t0 = Time.time; dur = Mathf.Max(0.05f, time);
        }

        void Update()
        {
            float k = Mathf.Clamp01((Time.time - t0) / dur);
            float e = k * k * (3f - 2f * k);
            if (fading)
            {
                alpha = fadeFrom * (1f - e);
                if (k >= 1f) { HideNow(); return; }
            }
            else if (t0 > -50f)
            {
                unroll = Mathf.Lerp(from, to, e);
                if (k >= 1f && to <= 0f) { HideNow(); return; }
            }
#if MR_TMP
            if (hint != null && hintOn && Time.time >= hintStart)
            {
                float ht = Time.time - hintStart;
                // fades in, then breathes very slightly (period 3 s)
                hintAlpha = Mathf.SmoothStep(0f, 1f, ht / 0.8f) * (0.85f + 0.15f * Mathf.Cos(ht * Mathf.PI * 2f / 3f));
                if (!hint.gameObject.activeSelf) hint.gameObject.SetActive(true);
                var c = hintColor; c.a *= hintAlpha * alpha;
                hint.color = c;
            }
#endif
            Apply();
        }

        void Apply()
        {
            if (paper == null) return;
            paper.enabled = alpha > 0.002f;
            paper.GetPropertyBlock(mpb);
            mpb.SetTexture("_FiberTex", fiberTex);
            mpb.SetTexture("_CloudTex", cloudTex);
            mpb.SetVector("_Size", new Vector4(size.x, size.y, size.x + margin * 2f, size.y + margin * 2f));
            mpb.SetFloat("_Unroll", unroll);
            mpb.SetFloat("_Alpha", alpha);
            mpb.SetFloat("_CloudGlow", CloudGlow);
            paper.SetPropertyBlock(mpb);
        }

        // ------------------------------------------------------------------ textures (made once)

        static void EnsureTextures()
        {
            if (fiberTex == null) fiberTex = MakeFibers(512, 7);
            if (cloudTex == null) cloudTex = MakeCloud(256);
        }

        /// <summary>R = fine paper fibres (mostly along the sheet), G = soft blotches, B = edge noise; all tileable.</summary>
        static Texture2D MakeFibers(int n, int seed)
        {
            var rng = new System.Random(seed);
            var fib = new float[n * n];
            for (int f = 0; f < 900; f++)
            {
                float x = (float)rng.NextDouble() * n, y = (float)rng.NextDouble() * n;
                float ang = ((float)rng.NextDouble() - 0.5f) * (rng.NextDouble() < 0.8 ? 0.8f : 3.1f);
                float len = 12f + (float)rng.NextDouble() * 60f, bend = ((float)rng.NextDouble() - 0.5f) * 0.04f;
                float w = 0.25f + (float)rng.NextDouble() * 0.75f;
                for (float s = 0f; s < len; s += 0.5f)
                {
                    ang += bend;
                    x += Mathf.Cos(ang) * 0.5f; y += Mathf.Sin(ang) * 0.5f;
                    int ix = ((int)x % n + n) % n, iy = ((int)y % n + n) % n;
                    fib[iy * n + ix] = Mathf.Max(fib[iy * n + ix], w);
                }
            }
            var blotch = PeriodicNoise(n, 6, seed + 1, 0.6f, PeriodicNoise(n, 14, seed + 2, 0.4f, null));
            var edge = PeriodicNoise(n, 24, seed + 3, 0.65f, PeriodicNoise(n, 64, seed + 4, 0.35f, null));
            var px = new Color32[n * n];
            for (int i = 0; i < px.Length; i++)
            {
                // soften the fibres a little with their neighbours (a cheap blur along x)
                int x = i % n, row = i - x;
                float f = fib[i] * 0.6f + (fib[row + (x + 1) % n] + fib[row + (x + n - 1) % n]) * 0.2f;
                px[i] = new Color32((byte)(Mathf.Clamp01(f) * 255), (byte)(Mathf.Clamp01(blotch[i]) * 255),
                                    (byte)(Mathf.Clamp01(edge[i]) * 255), 255);
            }
            var t = new Texture2D(n, n, TextureFormat.RGBA32, true, true) { name = "MR_HanjiFibres", wrapMode = TextureWrapMode.Repeat, filterMode = FilterMode.Bilinear, anisoLevel = 2 };
            t.SetPixels32(px);
            t.Apply(true, true);
            return t;
        }

        /// <summary>Smooth value noise on a <paramref name="cells"/>-cell periodic lattice, added (× amp) onto <paramref name="add"/>.</summary>
        static float[] PeriodicNoise(int n, int cells, int seed, float amp, float[] add)
        {
            var rng = new System.Random(seed);
            var lat = new float[cells * cells];
            for (int i = 0; i < lat.Length; i++) lat[i] = (float)rng.NextDouble();
            var o = add ?? new float[n * n];
            for (int y = 0; y < n; y++)
            for (int x = 0; x < n; x++)
            {
                float fx = (float)x / n * cells, fy = (float)y / n * cells;
                int x0 = (int)fx, y0 = (int)fy;
                float tx = fx - x0, ty = fy - y0;
                tx = tx * tx * (3 - 2 * tx); ty = ty * ty * (3 - 2 * ty);
                int x1 = (x0 + 1) % cells, y1 = (y0 + 1) % cells;
                float v = Mathf.Lerp(Mathf.Lerp(lat[y0 * cells + x0], lat[y0 * cells + x1], tx),
                                     Mathf.Lerp(lat[y1 * cells + x0], lat[y1 * cells + x1], tx), ty);
                o[y * n + x] += v * amp;
            }
            return o;
        }

        /// <summary>
        /// One corner's auspicious cloud (as a line drawing in R): a large curl with a smaller counter-curl tucked under
        /// it and a trailing wisp along the paper's edge. u runs towards the side edge, v towards the top / bottom edge.
        /// </summary>
        static Texture2D MakeCloud(int n)
        {
            var px = new float[n * n];
            float th = 0.028f; // line half-width (uv)
            void Stamp(Vector2 c, float w)
            {
                int r = Mathf.CeilToInt((w + 0.01f) * n);
                int cx = (int)(c.x * n), cy = (int)(c.y * n);
                for (int y = cy - r; y <= cy + r; y++)
                for (int x = cx - r; x <= cx + r; x++)
                {
                    if (x < 0 || y < 0 || x >= n || y >= n) continue;
                    float d = Vector2.Distance(new Vector2((x + 0.5f) / n, (y + 0.5f) / n), c);
                    float v = 1f - Mathf.SmoothStep(w * 0.55f, w, d);
                    if (v > px[y * n + x]) px[y * n + x] = v;
                }
            }
            void Spiral(Vector2 c, float r0, float a0, float turns, float dir, float w0)
            {
                int steps = (int)(turns * 160);
                for (int i = 0; i <= steps; i++)
                {
                    float t = (float)i / steps;
                    float a = a0 + dir * t * turns * Mathf.PI * 2f;
                    float r = r0 * Mathf.Lerp(1f, 0.18f, t);
                    Stamp(c + new Vector2(Mathf.Cos(a), Mathf.Sin(a)) * r, w0 * Mathf.Lerp(1f, 0.6f, t));
                }
            }
            void Curve(Vector2 a, Vector2 b, Vector2 c, float w0, float w1)
            {
                for (int i = 0; i <= 200; i++)
                {
                    float t = i / 200f;
                    var p = Vector2.Lerp(Vector2.Lerp(a, b, t), Vector2.Lerp(b, c, t), t);
                    Stamp(p, Mathf.Lerp(w0, w1, t));
                }
            }
            // big curl hugging the corner, a smaller counter-curl below-inside it, and a wisp running inward
            Spiral(new Vector2(0.62f, 0.6f), 0.27f, Mathf.PI * 1.5f, 1.35f, 1f, th);
            Spiral(new Vector2(0.3f, 0.36f), 0.16f, Mathf.PI * 0.2f, 1.2f, -1f, th * 0.85f);
            Curve(new Vector2(0.62f, 0.33f), new Vector2(0.42f, 0.12f), new Vector2(0.04f, 0.1f), th * 0.9f, th * 0.3f);
            Curve(new Vector2(0.35f, 0.6f), new Vector2(0.2f, 0.82f), new Vector2(0.04f, 0.8f), th * 0.7f, th * 0.25f);
            var c32 = new Color32[n * n];
            for (int i = 0; i < c32.Length; i++) { byte b = (byte)(Mathf.Clamp01(px[i]) * 255); c32[i] = new Color32(b, b, b, b); }
            var tex = new Texture2D(n, n, TextureFormat.RGBA32, true, true) { name = "MR_CloudMotif", wrapMode = TextureWrapMode.Clamp, filterMode = FilterMode.Trilinear };
            tex.SetPixels32(c32);
            tex.Apply(true, true);
            return tex;
        }
    }
}
