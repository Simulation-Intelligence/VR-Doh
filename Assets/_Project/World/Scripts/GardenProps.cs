using UnityEngine;
using UnityEngine.Rendering;

namespace MoonRabbit.World
{
    /// <summary>
    /// Life for the team's garden props (placed by the builder): the paper lanterns light one by one with progress
    /// (paper emission through a property block + a soft halo at each lantern, no real-time lights), the wind chime
    /// sways a little, and the firefly swarms drift slowly and glow brighter as the night comes alive.
    /// </summary>
    public class GardenProps : MonoBehaviour
    {
        public Renderer[] lanternStands;
        [Tooltip("Lantern centres relative to the environment (filled by the builder).")]
        public Vector3[] lanternPoints;
        [Tooltip("First lantern lights at this progress, the next ones every 'lanternStep'.")]
        public float lanternStart = 0.12f;
        public float lanternStep = 0.12f;
        public Color paperGlowColor = new Color(1f, 0.71f, 0.36f);
        public float paperGlowMax = 1.6f;
        public float paperEmber = 0.05f;
        public Transform[] chimes;
        public Transform[] fireflies;
        public Renderer[] fireflyRenderers;
        public Color fireflyGlowColor = new Color(1f, 0.89f, 0.48f);
        public Vector2 fireflyGlow = new Vector2(0.4f, 2.2f);
        public Material haloMaterial;
        public Mesh quad;
        public float lanternHaloSize = 0.75f;

        MoonStory story;
        MaterialPropertyBlock mpb;
        Material halo;
        Matrix4x4[] haloM;
        Vector3[] flyBase;
        Quaternion[] chimeBase;
        int paperSub = -1, flySub = -1;

        void Start()
        {
            story = GetComponentInParent<Transform>().root.GetComponentInChildren<MoonStory>();
            if (story == null) story = FindAnyObjectByType<MoonStory>();
            mpb = new MaterialPropertyBlock();
            if (haloMaterial != null) halo = new Material(haloMaterial) { name = "MR_PropLanternHalo (runtime)", enableInstancing = true };
            haloM = new Matrix4x4[lanternPoints != null ? lanternPoints.Length : 0];
            flyBase = new Vector3[fireflies != null ? fireflies.Length : 0];
            for (int i = 0; i < flyBase.Length; i++) if (fireflies[i] != null) flyBase[i] = fireflies[i].localPosition;
            chimeBase = new Quaternion[chimes != null ? chimes.Length : 0];
            for (int i = 0; i < chimeBase.Length; i++) if (chimes[i] != null) chimeBase[i] = chimes[i].localRotation;
            if (lanternStands != null && lanternStands.Length > 0 && lanternStands[0] != null) paperSub = Find(lanternStands[0], "MR_PropPaperGlow");
            if (fireflyRenderers != null && fireflyRenderers.Length > 0 && fireflyRenderers[0] != null) flySub = Find(fireflyRenderers[0], "MR_PropFireflyGlow");
        }

        static int Find(Renderer r, string mat)
        {
            var m = r.sharedMaterials;
            for (int i = 0; i < m.Length; i++) if (m[i] != null && m[i].name == mat) return i;
            return -1;
        }

        void OnDestroy() { if (halo != null) Destroy(halo); }

        static float Ramp(float v, float a, float b) { float t = Mathf.Clamp01((v - a) / Mathf.Max(1e-4f, b - a)); return t * t * (3f - 2f * t); }

        void Update()
        {
            float v = story != null ? story.ShownProgress : 0f;
            float bright = story != null ? story.Brightness : 0f;
            float t = Time.time;

            // paper lanterns, one after another
            int n = lanternPoints != null ? lanternPoints.Length : 0, k = 0;
            float lit = 0f;
            var cam = MoonStory.MainCamera();
            for (int i = 0; i < n; i++)
            {
                float on = Ramp(v, lanternStart + lanternStep * i, lanternStart + lanternStep * i + 0.08f);
                lit += on;
                if (on <= 0.01f || cam == null) continue;
                var p = transform.root.TransformPoint(lanternPoints[i]);
                float flicker = 1f + 0.05f * Mathf.Sin(t * 7.3f + i * 2.1f) + 0.03f * Mathf.Sin(t * 13.1f + i);
                haloM[k++] = Matrix4x4.TRS(p, Quaternion.LookRotation(p - cam.transform.position), Vector3.one * lanternHaloSize * on * flicker);
            }
            if (paperSub >= 0 && lanternStands != null)
            {
                float glow = Mathf.Lerp(paperEmber, paperGlowMax, n > 0 ? lit / n : 0f);
                foreach (var r in lanternStands)
                {
                    if (r == null) continue;
                    r.GetPropertyBlock(mpb, paperSub);
                    mpb.SetColor("_EmissionColor", paperGlowColor * glow);
                    r.SetPropertyBlock(mpb, paperSub);
                }
            }
            if (halo != null && quad != null && k > 0)
            {
                halo.SetFloat("_Intensity", 0.55f);
                Graphics.RenderMeshInstanced(new RenderParams(halo)
                {
                    layer = gameObject.layer,
                    worldBounds = new Bounds(transform.position, Vector3.one * 60f),
                    shadowCastingMode = ShadowCastingMode.Off,
                    receiveShadows = false,
                }, quad, 0, haloM, k);
            }

            // wind chime: a slow, small sway (pivot at the top)
            for (int i = 0; i < chimeBase.Length; i++)
            {
                if (chimes[i] == null) continue;
                float a = 2.5f * Mathf.Sin(t * 0.9f + i) + 1.2f * Mathf.Sin(t * 2.3f + i * 1.7f);
                float b = 1.8f * Mathf.Sin(t * 0.7f + 2f + i);
                chimes[i].localRotation = chimeBase[i] * Quaternion.Euler(a, 0f, b);
            }

            // fireflies: drift on slow loops and glow brighter with the night
            for (int i = 0; i < flyBase.Length; i++)
            {
                if (fireflies[i] == null) continue;
                fireflies[i].localPosition = flyBase[i] + new Vector3(Mathf.Sin(t * 0.21f + i * 2f) * 0.6f,
                                                                      Mathf.Sin(t * 0.33f + i) * 0.25f,
                                                                      Mathf.Cos(t * 0.17f + i * 3f) * 0.6f);
                fireflies[i].localRotation = Quaternion.Euler(0f, t * 6f + i * 97f, 0f);
                var r = fireflyRenderers != null && i < fireflyRenderers.Length ? fireflyRenderers[i] : null;
                if (r != null && flySub >= 0)
                {
                    float tw = 0.75f + 0.25f * Mathf.Sin(t * 3.7f + i * 1.9f);
                    r.GetPropertyBlock(mpb, flySub);
                    mpb.SetColor("_EmissionColor", fireflyGlowColor * Mathf.Lerp(fireflyGlow.x, fireflyGlow.y, bright) * tw);
                    r.SetPropertyBlock(mpb, flySub);
                }
            }
        }
    }
}
