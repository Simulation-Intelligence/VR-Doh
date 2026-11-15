using UnityEngine;
using UnityEngine.Rendering;

namespace MoonRabbit.World
{
    /// <summary>
    /// World-space story line for VR: placed once in front of the head (never head-locked), soft warm-white letters
    /// with a dark drop shadow and a faint warm glow behind, so it reads over the sky and the bright Moon.
    /// Built-in TextMesh (no TextMeshPro dependency); fading only changes vertex colours and a property block.
    /// </summary>
    public class StoryText : MonoBehaviour
    {
        TextMesh main, shadow;
        Renderer glow;
        MaterialPropertyBlock mpb;
        Color color;
        float glowIntensity;
        public Color glowColor = new Color(1f, 0.74f, 0.45f);

        /// <summary>Glyph line height of LegacyRuntime at fontSize 96 is 11 × characterSize.</summary>
        const float LineHeightPerCharSize = 11f;

        public static StoryText Create(Transform parent, string name, float letterHeight, Color color,
                                       Material haloMaterial, Mesh quad, float glowIntensity = 0.12f)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, true);
            var st = go.AddComponent<StoryText>();
            st.color = color;
            st.glowIntensity = glowIntensity;
            st.mpb = new MaterialPropertyBlock();
            var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            st.main = MakeText(go.transform, "Text", font, letterHeight, Vector3.zero);
            st.shadow = MakeText(go.transform, "Shadow", font, letterHeight, new Vector3(0.006f, -0.006f, 0.004f) * (letterHeight / 0.13f));
            if (haloMaterial != null && quad != null)
            {
                var g = new GameObject("Glow");
                g.transform.SetParent(go.transform, false);
                g.transform.localPosition = new Vector3(0f, 0f, 0.02f);
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

        static TextMesh MakeText(Transform parent, string name, Font font, float letterHeight, Vector3 offset)
        {
            var g = new GameObject(name);
            g.transform.SetParent(parent, false);
            g.transform.localPosition = offset;
            var t = g.AddComponent<TextMesh>();
            t.font = font;
            t.fontSize = 96;
            t.characterSize = letterHeight / LineHeightPerCharSize;
            t.anchor = TextAnchor.MiddleCenter;
            t.alignment = TextAlignment.Center;
            var r = g.GetComponent<MeshRenderer>();
            if (font != null) r.sharedMaterial = font.material;
            r.shadowCastingMode = ShadowCastingMode.Off;
            r.receiveShadows = false;
            return t;
        }

        public void SetText(string text)
        {
            main.text = text;
            shadow.text = text;
            if (glow != null)
            {
                // glow behind the line: as wide as the text, a few lines tall
                var size = main.GetComponent<MeshRenderer>().bounds.size;
                float w = Mathf.Max(size.x, size.z);
                glow.transform.localScale = new Vector3(w * 1.3f + 0.3f, main.characterSize * LineHeightPerCharSize * 3.2f, 1f);
            }
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
            a = Mathf.Clamp01(a);
            main.color = new Color(color.r, color.g, color.b, a);
            shadow.color = new Color(0f, 0f, 0f, a * 0.6f);
            if (glow != null)
            {
                glow.GetPropertyBlock(mpb);
                mpb.SetFloat("_Intensity", glowIntensity * a);
                mpb.SetColor("_Color", glowColor);
                glow.SetPropertyBlock(mpb);
                glow.enabled = a > 0.001f;
            }
            if (a > 0.001f && !gameObject.activeSelf) gameObject.SetActive(true);
            if (a <= 0.001f && gameObject.activeSelf) gameObject.SetActive(false);
        }
    }
}
