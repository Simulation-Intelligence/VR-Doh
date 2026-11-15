using UnityEngine;
using UnityEngine.Rendering;

namespace MoonRabbit.World
{
    /// <summary>
    /// Soft star mist drifting slowly around and below the floating island: a few large additive halo quads
    /// (MoonRabbit/Glow), camera-facing, one instanced draw call. Brightens a little as the night comes alive
    /// (MoonStory.ShownProgress). Runtime only; nothing in the scene changes.
    /// </summary>
    [ExecuteAlways]
    public class SkyMist : MonoBehaviour
    {
        public Material material;
        public Mesh quad;
        [Range(0, 64)] public int count = 22;
        [Tooltip("Distance from the island's centre (m).")]
        public Vector2 radius = new Vector2(17f, 35f);
        [Tooltip("Height relative to the island's ground (m).")]
        public Vector2 height = new Vector2(-9f, -1.5f);
        public Vector2 size = new Vector2(7f, 15f);
        [Tooltip("Orbit speed (deg/s).")]
        public float drift = 0.6f;
        public Vector2 intensity = new Vector2(0.16f, 0.62f);
        public int seed = 9;

        Vector4[] puffs; // angle (deg), radius, height, size
        float[] phase;
        Matrix4x4[] m;
        Material runtimeMat;
        MoonStory story;

        void OnEnable()
        {
            var rng = new System.Random(seed);
            puffs = new Vector4[count];
            phase = new float[count];
            for (int i = 0; i < count; i++)
            {
                puffs[i] = new Vector4((float)rng.NextDouble() * 360f,
                                       Mathf.Lerp(radius.x, radius.y, (float)rng.NextDouble()),
                                       Mathf.Lerp(height.x, height.y, (float)rng.NextDouble()),
                                       Mathf.Lerp(size.x, size.y, (float)rng.NextDouble()));
                phase[i] = (float)rng.NextDouble() * 10f;
            }
            m = new Matrix4x4[count];
            Camera.onPreCull += Draw;
        }

        void OnDisable()
        {
            Camera.onPreCull -= Draw;
            if (runtimeMat != null) { if (Application.isPlaying) Destroy(runtimeMat); else DestroyImmediate(runtimeMat); }
        }

        void Draw(Camera cam)
        {
            if (material == null || quad == null || count == 0 || puffs == null) return;
            if (cam.cameraType == CameraType.Preview || cam.cameraType == CameraType.Reflection) return;
            if (story == null && Application.isPlaying) story = FindAnyObjectByType<MoonStory>();
            float p = story != null ? story.Brightness : 1f;
            if (runtimeMat == null) runtimeMat = new Material(material) { hideFlags = HideFlags.DontSave, enableInstancing = true };
            runtimeMat.SetFloat("_Intensity", Mathf.Lerp(intensity.x, intensity.y, p));

            var c = transform.position;
            var eye = cam.transform.position;
            float t = Application.isPlaying ? Time.time : 0f;
            for (int i = 0; i < count; i++)
            {
                var q = puffs[i];
                float a = (q.x + t * drift * (i % 2 == 0 ? 1f : 0.7f)) * Mathf.Deg2Rad;
                var pos = c + new Vector3(Mathf.Sin(a) * q.y, q.z + Mathf.Sin(t * 0.13f + phase[i]) * 0.6f, Mathf.Cos(a) * q.y);
                float breathe = 1f + 0.08f * Mathf.Sin(t * 0.2f + phase[i]);
                m[i] = Matrix4x4.TRS(pos, Quaternion.LookRotation(pos - eye), new Vector3(q.w * 1.6f, q.w, 1f) * breathe);
            }
            var rp = new RenderParams(runtimeMat)
            {
                camera = cam,
                layer = gameObject.layer,
                worldBounds = new Bounds(c, Vector3.one * (radius.y * 2f + size.y * 2f)),
                shadowCastingMode = ShadowCastingMode.Off,
                receiveShadows = false,
            };
            Graphics.RenderMeshInstanced(rp, quad, 0, m, count);
        }
    }
}
