using System;
using UnityEngine;
using UnityEngine.Rendering;
using Object = UnityEngine.Object;
#if MR_TMP
using TMPro;
#endif

namespace MoonRabbit.World
{
    /// <summary>Look of a story line: font, material preset (outline / shadow / glow), colour, size and spacing.</summary>
    [Serializable]
    public class StoryTextStyle
    {
        [Tooltip("TMP_FontAsset (Object so this works without a TMP reference).")]
        public Object font;
        public Material material;
        public Color color = new Color(0.97f, 0.95f, 0.9f);
        [Tooltip("Capital letter height (m) at the text distance.")]
        public float letterHeight = 0.1f;
        public float characterSpacing;
        public float lineSpacing;
        public Color glowColor = new Color(1f, 0.74f, 0.45f);
        public float glowIntensity = 0.14f;
        [Tooltip("Letters rise one by one from below along a slight curve as the line appears (legend-like).")]
        public bool rise = true;
        [Tooltip("One gentle swell and glow pulse once the line has risen (for instructions).")]
        public bool pulse;
        [Tooltip("Glow behind the line breathes slowly (period ~2 s, small amplitude) and flashes once when it appears.")]
        public bool breathe;
    }

    /// <summary>
    /// World-space story line for VR: placed once in front of the head (never head-locked), letters with a dark outline,
    /// soft drop shadow and warm glow (TextMesh Pro SDF material), plus a wide warm halo behind the line so it reads
    /// over the sky and the bright Moon. Lines can rise in letter by letter along a gentle curve, and pulse once.
    /// Fading only changes vertex alpha and a property block. Without TextMesh Pro it falls back to a TextMesh.
    /// </summary>
    public class StoryText : MonoBehaviour
    {
        [Header("Rise-in (letter by letter, like a legend being told)")]
        [Tooltip("Delay between letters (s); the line's total is kept within maxReveal.")]
        public float letterDelay = 0.035f;
        [Tooltip("Extra pause at a space, so the line flows word by word (s).")]
        public float wordPause = 0.07f;
        [Tooltip("Longest time the whole line takes to rise in (s); long lines use shorter letter delays.")]
        public float maxReveal = 1.8f;
        [Tooltip("Time one letter takes to rise into place (s), ease-out.")]
        public float riseTime = 0.7f;
        [Tooltip("A letter starts this far below its place (m) and this far to the side, on a soft arc.")]
        public float riseDistance = 0.2f;
        public float riseSide = 0.06f;
        [Tooltip("Start scale and tilt (deg) of a rising letter; both unwind as it settles.")]
        public float riseScale = 0.85f;
        public float riseTilt = 12f;
        [Header("Drift-out (like incense smoke)")]
        public float exitTime = 0.8f;
        public float exitRise = 0.1f;
        public float exitStagger = 0.012f;

        StoryTextStyle style;
        Renderer glow;
        MaterialPropertyBlock mpb;
        float alpha, revealStart = -100f, pulseStart = -100f, exitStart = -100f, appearStart = -100f;
        bool revealing, exiting;
        float swellNow;
        int pulseFrom = -1; // first character of the emphasised part (pulses on its own), -1 = whole line
        float[] charStart;
        float revealDuration;
#if MR_TMP
        TextMeshPro tmp;
        float fontSizeForHeight = -1f;
        Vector3[][] baseVerts;
        Color32[][] baseColors;
#endif
        TextMesh main, shadow;

        /// <summary>Line height of LegacyRuntime at fontSize 96 is 11 × characterSize (fallback only).</summary>
        const float LineHeightPerCharSize = 11f;

        public static StoryText Create(Transform parent, string name, StoryTextStyle style, Material haloMaterial, Mesh quad)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, true);
            var st = go.AddComponent<StoryText>();
            st.style = style;
            st.mpb = new MaterialPropertyBlock();
#if MR_TMP
            var t = new GameObject("Text");
            t.transform.SetParent(go.transform, false);
            st.tmp = t.AddComponent<TextMeshPro>();
            if (style.font is TMP_FontAsset fa) st.tmp.font = fa;
            if (style.material != null) st.tmp.fontSharedMaterial = style.material;
            st.tmp.alignment = TextAlignmentOptions.Center;
            st.tmp.textWrappingMode = TextWrappingModes.NoWrap;
            st.tmp.characterSpacing = style.characterSpacing;
            st.tmp.lineSpacing = style.lineSpacing;
            st.tmp.color = style.color;
            st.tmp.rectTransform.sizeDelta = new Vector2(20f, 5f);
            var tr = st.tmp.GetComponent<MeshRenderer>();
            tr.shadowCastingMode = ShadowCastingMode.Off;
            tr.receiveShadows = false;
#else
            var legacy = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            st.main = MakeText(go.transform, "Text", legacy, style.letterHeight * 1.4f, Vector3.zero);
            st.shadow = MakeText(go.transform, "Shadow", legacy, style.letterHeight * 1.4f, new Vector3(0.006f, -0.006f, 0.004f));
#endif
            if (haloMaterial != null && quad != null)
            {
                var g = new GameObject("Glow");
                g.transform.SetParent(go.transform, false);
                g.transform.localPosition = new Vector3(0f, 0f, 0.03f);
                g.AddComponent<MeshFilter>().sharedMesh = quad;
                var r = g.AddComponent<MeshRenderer>();
                r.sharedMaterial = haloMaterial;
                r.shadowCastingMode = ShadowCastingMode.Off;
                r.receiveShadows = false;
                st.glow = r;
            }
            st.SetAlpha(0f);
            go.SetActive(false);
            return st;
        }

        /// <summary>Older signature (ending text): a style built from the arguments.</summary>
        public static StoryText Create(Transform parent, string name, float letterHeight, Color color,
                                       Material haloMaterial, Mesh quad, float glowIntensity = 0.12f,
                                       Object font = null, Material fontMaterial = null,
                                       float characterSpacing = 0f, float lineSpacing = 0f)
        {
            return Create(parent, name, new StoryTextStyle
            {
                font = font, material = fontMaterial, color = color, letterHeight = letterHeight,
                characterSpacing = characterSpacing, lineSpacing = lineSpacing, glowIntensity = glowIntensity, rise = false,
            }, haloMaterial, quad);
        }

        static TextMesh MakeText(Transform parent, string name, Font font, float lineHeight, Vector3 offset)
        {
            var g = new GameObject(name);
            g.transform.SetParent(parent, false);
            g.transform.localPosition = offset;
            var t = g.AddComponent<TextMesh>();
            t.font = font;
            t.fontSize = 96;
            t.characterSize = lineHeight / LineHeightPerCharSize;
            t.anchor = TextAnchor.MiddleCenter;
            t.alignment = TextAlignment.Center;
            var r = g.GetComponent<MeshRenderer>();
            if (font != null) r.sharedMaterial = font.material;
            r.shadowCastingMode = ShadowCastingMode.Off;
            r.receiveShadows = false;
            return t;
        }

        /// <summary>Sets the line. The rise-in (if the style has it) and the pulse start now.</summary>
        public void SetText(string text) => SetText(text, -1);

        /// <summary>Sets the line; characters from <paramref name="emphasisFrom"/> on (rich text: index in the visible
        /// characters) get the gold pulse once the line has risen, the rest does not.</summary>
        public void SetText(string text, int emphasisFrom)
        {
            pulseFrom = emphasisFrom;
            Vector2 size;
#if MR_TMP
            // Enable first: TMP rebuilds its mesh (all letters fully opaque) on OnEnable. Doing that here, before the
            // mesh is laid out and hidden below, means no frame ever shows the whole sentence before it rises in.
            if (!gameObject.activeSelf) gameObject.SetActive(true);
            if (fontSizeForHeight < 0f)
            {
                // calibrate once: font size that gives the requested capital height in metres
                tmp.fontSize = 10f;
                tmp.text = "H";
                tmp.ForceMeshUpdate(true);
                float h = tmp.textBounds.size.y;
                fontSizeForHeight = h > 1e-5f ? 10f * style.letterHeight / h : 1f;
                tmp.fontSize = fontSizeForHeight;
            }
            tmp.text = text;
            tmp.ForceMeshUpdate(true);
            size = tmp.textBounds.size;
            // keep the laid-out mesh; the rise-in and fades write positions and alpha from it every frame
            var info = tmp.textInfo;
            baseVerts = new Vector3[info.meshInfo.Length][];
            baseColors = new Color32[info.meshInfo.Length][];
            for (int m = 0; m < info.meshInfo.Length; m++)
            {
                baseVerts[m] = (Vector3[])info.meshInfo[m].vertices.Clone();
                baseColors[m] = (Color32[])info.meshInfo[m].colors32.Clone();
            }
            revealStart = Time.time;
            revealing = style.rise;
            exiting = false;
            ComputeLetterTimes(text);
            WriteVertices(Time.time); // same frame: every letter starts invisible (or at the current line alpha)
#else
            main.text = text;
            shadow.text = text;
            var b = main.GetComponent<MeshRenderer>().bounds.size;
            size = new Vector2(Mathf.Max(b.x, b.z), main.characterSize * LineHeightPerCharSize);
#endif
            if (glow != null)
                glow.transform.localScale = new Vector3(size.x * 1.3f + 0.4f, Mathf.Max(size.y, style.letterHeight) * 2.6f + 0.15f, 1f);
            revealStart = Time.time;
            appearStart = Time.time;
            revealing = style.rise;
            exiting = false;
            ComputeLetterTimes(text);
            pulseStart = style.pulse || pulseFrom >= 0 ? Time.time + revealDuration : -100f; // the gold pulse once the line has risen
        }

        /// <summary>Seconds until every letter of the current line has risen into place (0 without the rise-in).</summary>
        public float RevealDuration => style != null && style.rise ? revealDuration : 0f;

        /// <summary>Seconds the drift-out takes.</summary>
        public float ExitDuration => exitTime + exitStagger * (charStart != null ? charStart.Length : 0);

        /// <summary>Letters drift upward and thin out like incense smoke, one after another.</summary>
        public void BeginExit()
        {
            exiting = true;
            exitStart = Time.time;
        }

        /// <summary>Start time (relative to the line) of each letter: steady rhythm, a little pause between words.</summary>
        void ComputeLetterTimes(string text)
        {
#if MR_TMP
            var info = tmp.textInfo;
            int n = info.characterCount;
            charStart = new float[n];
            int spaces = 0, letters = 0;
            for (int i = 0; i < n; i++) { if (char.IsWhiteSpace(info.characterInfo[i].character)) spaces++; else letters++; }
            float budget = Mathf.Max(0.2f, maxReveal - riseTime - spaces * wordPause);
            float delay = Mathf.Clamp(budget / Mathf.Max(1, letters), 0.012f, letterDelay);
            float t = 0f;
            for (int i = 0; i < n; i++)
            {
                charStart[i] = t;
                t += char.IsWhiteSpace(info.characterInfo[i].character) ? wordPause : delay;
            }
            revealDuration = (n > 0 ? charStart[n - 1] : 0f) + riseTime;
#else
            charStart = new float[0];
            revealDuration = 0f;
#endif
        }

        /// <summary>Places the text <paramref name="distance"/> ahead of the head's yaw, <paramref name="above"/> over eye height, facing the head.</summary>
        public void PlaceInFront(Transform head, float distance, float above)
        {
            var fwd = head.forward; fwd.y = 0f;
            if (fwd.sqrMagnitude < 1e-4f) fwd = Vector3.forward;
            fwd.Normalize();
            var pos = head.position + fwd * distance + Vector3.up * above;
            transform.SetPositionAndRotation(pos, Quaternion.LookRotation(pos - head.position, Vector3.up));
        }

        public void SetAlpha(float a)
        {
            alpha = Mathf.Clamp01(a);
#if !MR_TMP
            main.color = new Color(style.color.r, style.color.g, style.color.b, alpha);
            shadow.color = new Color(0f, 0f, 0f, alpha * 0.6f);
#endif
            if (alpha > 0.001f && !gameObject.activeSelf)
            {
                gameObject.SetActive(true);
#if MR_TMP
                // OnEnable just made TMP rebuild the mesh fully opaque: rebuild now and write our alphas this frame
                if (tmp != null) tmp.ForceMeshUpdate(true);
#endif
            }
            if (alpha <= 0.001f && gameObject.activeSelf && !revealing) gameObject.SetActive(false);
            if (gameObject.activeSelf) Refresh();
        }

        void LateUpdate() => Refresh();

        void Refresh()
        {
            float now = Time.time;
            // one gentle swell (+8 %) and glow pulse for instructions
            float pu = now - pulseStart;
            float swell = pu >= 0f && pu < 0.9f ? Mathf.Sin(pu / 0.9f * Mathf.PI) : 0f;
            swellNow = swell;
            float glowK = 1f + (pulseFrom >= 0 ? 0.6f : 1.5f) * swell;
            if (style.breathe)
            {
                float ap = now - appearStart;
                float flash = ap >= 0f ? Mathf.Exp(-ap * 1.6f) * Mathf.Clamp01(ap / 0.25f) : 0f;   // one bright moment, then settles
                glowK *= 1f + 0.18f * Mathf.Sin(ap * Mathf.PI) + 1.6f * flash;                    // slow breathing, period 2 s
            }
#if MR_TMP
            if (tmp != null) tmp.transform.localScale = Vector3.one * (pulseFrom >= 0 ? 1f : 1f + 0.08f * swell);
            WriteVertices(now);
#endif
            if (glow != null)
            {
                glow.GetPropertyBlock(mpb);
                mpb.SetFloat("_Intensity", style.glowIntensity * alpha * glowK);
                mpb.SetColor("_Color", style.glowColor);
                glow.SetPropertyBlock(mpb);
                glow.enabled = alpha > 0.001f;
            }
        }

#if MR_TMP
        /// <summary>Per-letter position (rise along a curve) and alpha (line fade × letter fade-in), written into the mesh.</summary>
        void WriteVertices(float now)
        {
            if (tmp == null || baseVerts == null) return;
            var info = tmp.textInfo;
            if (info.meshInfo.Length != baseVerts.Length) return;
            float cap = style.letterHeight;
            bool anyMoving = false;
            for (int m = 0; m < info.meshInfo.Length; m++)
            {
                var verts = info.meshInfo[m].vertices;
                var cols = info.meshInfo[m].colors32;
                if (verts == null || verts.Length != baseVerts[m].Length) return;
                System.Array.Copy(baseVerts[m], verts, verts.Length);
                for (int v = 0; v < cols.Length; v++) { var c = baseColors[m][v]; c.a = (byte)(c.a * alpha); cols[v] = c; }
            }
            if (revealing || exiting || (pulseFrom >= 0 && swellNow > 0f))
            {
                // world units → the text's local units
                float sc = tmp.transform.lossyScale.y;
                float local = sc > 1e-6f ? 1f / sc : 1f;
                for (int i = 0; i < info.characterCount; i++)
                {
                    var ch = info.characterInfo[i];
                    if (!ch.isVisible) continue;
                    Vector3 off = Vector3.zero; float scale = 1f, tilt = 0f, fade = 1f;
                    if (revealing && charStart != null && i < charStart.Length)
                    {
                        float t = Mathf.Clamp01((now - revealStart - charStart[i]) / riseTime);
                        if (t < 1f) anyMoving = true;
                        float e = 1f - Mathf.Pow(1f - t, 3f);                    // ease out
                        float r = 1f - e;
                        // soft arc: from below and to the side, curving in (side settles before height)
                        off = new Vector3(-r * r * riseSide, -r * riseDistance, 0f);
                        scale = Mathf.Lerp(riseScale, 1f, e);
                        tilt = r * riseTilt;
                        fade = Mathf.SmoothStep(0f, 1f, t * 1.5f);
                    }
                    float bright = 0f;
                    if (pulseFrom >= 0 && i >= pulseFrom && swellNow > 0f)
                    {
                        scale *= 1f + 0.12f * swellNow;   // only the emphasised words swell and brighten
                        bright = 0.45f * swellNow;
                    }
                    if (exiting)
                    {
                        // drift up and thin out, letter after letter, like incense smoke
                        float t = Mathf.Clamp01((now - exitStart - i * exitStagger) / exitTime);
                        float e = t * t;
                        off += new Vector3(Mathf.Sin(i * 1.7f) * 0.015f * e, exitRise * e, 0f);
                        scale *= 1f + 0.08f * e;
                        tilt += Mathf.Sin(i * 2.3f) * 8f * e;
                        fade *= 1f - Mathf.SmoothStep(0f, 1f, t);
                    }
                    int mi = ch.materialReferenceIndex, vi = ch.vertexIndex;
                    var verts = info.meshInfo[mi].vertices;
                    var cols = info.meshInfo[mi].colors32;
                    var centre = (verts[vi] + verts[vi + 2]) * 0.5f;
                    var rot = Quaternion.Euler(0f, 0f, tilt);
                    var offLocal = off * local;
                    for (int k = 0; k < 4; k++)
                    {
                        verts[vi + k] = centre + rot * ((verts[vi + k] - centre) * scale) + offLocal;
                        var c = cols[vi + k]; c.a = (byte)(c.a * fade);
                        if (bright > 0f) c = Color32.Lerp(c, new Color32(255, 248, 225, c.a), bright);
                        cols[vi + k] = c;
                    }
                }
                if (revealing && !anyMoving) revealing = false;
            }
            tmp.UpdateVertexData(TMP_VertexDataUpdateFlags.Vertices | TMP_VertexDataUpdateFlags.Colors32);
        }
#endif
    }
}
