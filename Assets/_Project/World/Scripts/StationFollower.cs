using UnityEngine;

namespace MoonRabbit.World
{
    /// <summary>
    /// Keeps the garden's usu props under the team's MochiStation when it moves at runtime (e.g. UsuTablePlacer puts it
    /// on a real table from the room scan): the pedestal moves under the station and stretches or shrinks so its top
    /// meets the station's height, and the warm dough light keeps its offset to the dough. Rice grains, strike
    /// detection and the ending's start already follow the live dough/usu (MochiLink). The station is found by type
    /// name (MochiStation), nothing of it is changed.
    /// </summary>
    [RequireComponent(typeof(MochiLink))]
    public class StationFollower : MonoBehaviour
    {
        public string stationType = "MochiStation";
        public Transform pedestal;
        [Tooltip("Pedestal height as built (m): its top is where the station stands by default.")]
        public float pedestalHeight = 0.8f;
        public Transform warmLight;
        [Tooltip("Warm light position relative to the station (filled by the builder).")]
        public Vector3 warmLightOffset = new Vector3(-0.32f, 0.67f, 0.22f);
        [Tooltip("Ground height under the pedestal, relative to the environment (the clearing is nearly flat).")]
        public float groundY = 0f;
        [Tooltip("Smallest / largest pedestal scale (a very low or very high table).")]
        public Vector2 heightScaleRange = new Vector2(0.3f, 2f);

        Transform station;
        MochiLink link;

        void Awake() => link = GetComponent<MochiLink>();
        Vector3 lastPos = new Vector3(float.NaN, 0f, 0f);
        float nextSearch;

        void LateUpdate()
        {
            if (station == null)
            {
                if (Time.unscaledTime < nextSearch) return;
                nextSearch = Time.unscaledTime + 1f;
                foreach (var mb in FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
                    if (mb.GetType().Name == stationType) { station = mb.transform; break; }
                if (station == null) return;
            }
            var p = station.position;
            if ((p - lastPos).sqrMagnitude < 1e-8f) return;
            bool firstFrame = float.IsNaN(lastPos.x);
            lastPos = p;
            // placed on a real table (or moved): stand the kine back at its rest beside the usu, unless someone holds it
            if (!firstFrame && link != null) link.RestKine();

            float ground = transform.position.y + groundY;
            if (pedestal != null)
            {
                pedestal.position = new Vector3(p.x, ground, p.z);
                float k = Mathf.Clamp((p.y - ground) / Mathf.Max(0.05f, pedestalHeight), heightScaleRange.x, heightScaleRange.y);
                var s = pedestal.localScale;
                pedestal.localScale = new Vector3(s.x, k, s.z);
            }
            if (warmLight != null)
            {
                warmLight.position = p + warmLightOffset;
                var aim = p + Vector3.up * 0.05f - warmLight.position;
                if (aim.sqrMagnitude > 1e-6f) warmLight.rotation = Quaternion.LookRotation(aim);
            }
        }
    }
}
