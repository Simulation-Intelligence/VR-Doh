using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;

namespace MoonRabbit.World
{
    /// <summary>
    /// Sky, fog and ambient light live in the scene's Lighting settings, which only the
    /// Builder edits. This component carries them inside the environment prefab and applies
    /// them when the prefab is enabled, so placing the prefab is enough.
    /// </summary>
    [ExecuteAlways, DisallowMultipleComponent]
    public class EnvironmentAtmosphere : MonoBehaviour
    {
        public Material skybox;
        public Light moonlight;

        [Header("Fog (ink-wash depth)")]
        public bool fog = true;
        public FogMode fogMode = FogMode.ExponentialSquared;
        public Color fogColor = new Color32(0x1E, 0x2A, 0x4A, 0xFF);
        public float fogDensity = 0.04f;
        public float fogStart = 6f;
        public float fogEnd = 40f;

        [Header("Ambient (trilight)")]
        public Color ambientSky = new Color(0.30f, 0.36f, 0.52f);
        public Color ambientEquator = new Color(0.20f, 0.24f, 0.36f);
        public Color ambientGround = new Color(0.10f, 0.11f, 0.16f);

        [Tooltip("Also apply in the Editor (Scene view preview). Turn off to leave scene lighting alone.")]
        public bool applyInEditMode = true;

        void OnEnable()
        {
            if (Application.isPlaying) { Apply(); return; }
            // Edit mode: only touch the lighting of the scene being edited, never while the
            // builder works in a preview scene or the prefab is open in Prefab Mode.
            if (applyInEditMode && gameObject.scene == SceneManager.GetActiveScene()) Apply();
        }

        [ContextMenu("Apply Now")]
        public void Apply()
        {
            if (skybox != null) RenderSettings.skybox = skybox;
            RenderSettings.fog = fog;
            RenderSettings.fogMode = fogMode;
            RenderSettings.fogColor = fogColor;
            RenderSettings.fogDensity = fogDensity;
            RenderSettings.fogStartDistance = fogStart;
            RenderSettings.fogEndDistance = fogEnd;
            RenderSettings.ambientMode = AmbientMode.Trilight;
            RenderSettings.ambientSkyColor = ambientSky;
            RenderSettings.ambientEquatorColor = ambientEquator;
            RenderSettings.ambientGroundColor = ambientGround;
            if (moonlight != null) RenderSettings.sun = moonlight;
        }
    }
}
