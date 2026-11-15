using System.Collections;
using UnityEngine;
using UnityEngine.Rendering;

namespace MoonRabbit.World
{
    /// <summary>
    /// The night comes alive with <see cref="MoonProgress"/> (eased, about a second behind the real value):
    /// the Moon waxes from a dark crescent to a bright full Moon and the garden brightens with it (moonlight,
    /// ambient, thinning fog); stone lanterns light one by one from 25 %, cassia blossoms swell and glow from 50 %,
    /// the stepping stones glow one by one toward the sky from 75 %, the bamboo turns silvery at the end.
    /// Every strike gives the Moon a small pulse ("that strike lit the Moon").
    /// At 1 the ending plays: the mochi rises from the usu and flies, leaving a sparkling trail, to the Earth;
    /// the Earth glows and its lights come on; "Happy Chuseok, Earth!" fades in; after <see cref="endingDuration"/>
    /// everything resets. Runtime only: property blocks, RenderSettings, light intensities; no real-time lights added.
    /// </summary>
    [RequireComponent(typeof(MoonProgress), typeof(MochiLink))]
    public class MoonStory : MonoBehaviour
    {
        [Tooltip("Off: the environment stays as built (full Moon), MoonProgress still counts.")]
        public bool storyEnabled = true;
        [Tooltip("Seconds the visuals take to ease to a new progress value (SmoothDamp time).")]
        public float smoothTime = 0.35f;

        [Header("Moon")]
        [Tooltip("On: the Moon waxes and brightens with progress. Off: it stays a dark crescent until the mochi reaches the Earth.")]
        public bool moonFollowsProgress = true;
        public Renderer moon;
        public Renderer moonGlow;
        [Tooltip("Phase angle at progress 0 (deg): 180 = new Moon, 90 = half, 0 = full.")]
        public float crescentAngle = 132f;
        public Vector2 moonEmission = new Vector2(0.03f, 0.35f);
        [Tooltip("Brightness of the unlit part (earthshine), at progress 0 and 1 (linear colour space: 0.12 already reads mid grey).")]
        public Vector2 moonDarkSide = new Vector2(0.02f, 0.12f);
        public Vector2 moonGlowIntensity = new Vector2(0.25f, 1.15f);
        public Vector2 moonGlowScale = new Vector2(3.0f, 5.0f);
        [Tooltip("Extra Moon brightness right after a strike (decays in about half a second).")]
        public float strikePulse = 0.35f;

        [Header("Moonlight, ambient, fog (multipliers of the built values)")]
        public Light moonlight;
        public EnvironmentAtmosphere atmosphere;
        public Vector2 moonlightScale = new Vector2(0.6f, 1.25f);
        public Vector2 ambientScale = new Vector2(0.62f, 1.3f);
        public Vector2 fogScale = new Vector2(1.5f, 0.45f);

        [Header("Stone lanterns (25 %: one by one)")]
        public Renderer[] lanterns;
        public int lanternGlowMaterial = 1;
        public Light[] lanternLights;
        public Renderer[] lanternPools;
        public float lanternStart = 0.25f;
        public float lanternStep = 0.12f;
        [Tooltip("Glow of an unlit lantern (a faint ember), fraction of the lit glow.")]
        public float lanternEmber = 0.1f;

        [Header("Cassia blossoms (50 %: grow and glow)")]
        public Renderer cassiaTree;
        public int blossomMaterial = 2;
        [Tooltip("Blossom positions relative to this environment (filled by the builder).")]
        public Vector3[] blossomPoints;
        public float blossomStart = 0.5f;
        public Color blossomGlowColor = new Color(1f, 0.78f, 0.35f);
        public float blossomHaloSize = 0.34f;
        public Vector2 blossomEmission = new Vector2(0.45f, 2.2f);

        [Header("Stepping stones (75 %: one by one toward the sky)")]
        public Vector3[] stonePoints;
        public float stoneStart = 0.75f;
        public Color stoneGlowColor = new Color(0.72f, 0.82f, 1f);
        public float stoneHaloSize = 0.95f;

        [Header("Bamboo (100 %: silver)")]
        public BambooGrove bamboo;
        public Color bambooSilver = new Color(0.72f, 0.8f, 0.86f);
        [Range(0f, 1f)] public float bambooSilverAmount = 0.45f;

        [Header("Ending: the mochi goes to the Earth")]
        public Renderer earth;
        public Renderer earthGlow;
        public Material haloMaterial;
        public Material orbMaterial;
        public Mesh quadMesh;
        public Mesh sphereMesh;
        public Color orbGlowColor = new Color(1f, 0.93f, 0.8f);
        public float orbRadius = 0.085f;
        [Tooltip("While it flies away the orb keeps at least this angular radius (tan of the angle).")]
        public float orbMinAngularRadius = 0.022f;
        public float riseTime = 1.2f;
        public float flyTime = 3.6f;
        [Range(8, 128)] public int trailLength = 48;
        public Color trailColor = new Color(1f, 0.86f, 0.62f);
        [Range(0, 80)] public int earthLightCount = 36;
        public Color earthLightColor = new Color(1f, 0.8f, 0.45f);
        [Tooltip("Seconds from completion to the automatic reset.")]
        public float endingDuration = 12f;
        [Range(0, 60)] public int wishCount = 30;
        public Color wishColor = new Color(1f, 0.74f, 0.42f);
        [Tooltip("Seconds a wish light takes to rise from the Earth toward the Moon.")]
        public float wishRiseTime = 7f;
        public string endingText = "Happy Chuseok, Earth!";
        public Color textColor = new Color(0.97f, 0.95f, 0.9f);
        [Tooltip("Letter height of story text (m) and its distance from the head.")]
        public float textHeight = 0.14f;
        public float textDistance = 2.8f;
        public float textAboveEyes = 0.25f;

        public float ShownProgress { get; private set; }
        public bool InEnding { get; private set; }
        /// <summary>0..1 extra brightness of the Earth after the mochi arrives.</summary>
        public float EarthPulse { get; private set; }
        /// <summary>Hook for the intro / ending to darken or light the Earth (0 = dark night side, 1 = built look).</summary>
        public float EarthBrightness { get; set; } = 1f;
        public Material HaloMaterialFor(string n, Color c) => HaloCopy(n, c);
        public Material TextHalo => textHalo;
        /// <summary>Fired after the ending's reset (the intro restarts from here).</summary>
        public event System.Action Resetted;

        MoonProgress progress;
        MochiLink link;
        MaterialPropertyBlock mpb;
        Material blossomHalo, stoneHalo, orbHalo, trailHalo, cityHalo, textHalo, burstHalo, wishHalo;
        Matrix4x4[] blossomM, stoneM, trailM, cityM;
        Vector3[] trailPos, cityDir;
        float[] trailTime;
        int trailHead, trailCount;
        float trailUntil = -1f, cityOn, burstStart = -10f, wishStart = -100f;
        Matrix4x4[] wishM;
        Matrix4x4[] burstM = new Matrix4x4[24];
        Color[] lanternBaseEmission;
        float[] lanternLightBase, poolBase;
        Color blossomBaseEmission;
        float moonlightBase = 1f, moonGlowBaseScale = 4.2f, earthGlowBase = 0.5f, earthGlowBaseScale = 3.2f, earthEmissionBase = 0.45f, earthAmbientBase = 0.18f;
        float velocity, pulse, moonFull;
        Vector3 earthLightBase = Vector3.right;
        GameObject orb;
        Renderer orbRenderer;
        StoryText text;
        bool started;

        /// <summary>Optional step between the Earth lighting up and the text (wish lights). Set by StoryIntro.</summary>
        public System.Func<IEnumerator> afterEarthLit;

        void Awake()
        {
            progress = GetComponent<MoonProgress>();
            link = GetComponent<MochiLink>();
            mpb = new MaterialPropertyBlock();
        }

        void Start()
        {
            if (moonlight != null) moonlightBase = moonlight.intensity;
            if (moonGlow != null) moonGlowBaseScale = moonGlow.transform.localScale.x;
            if (earthGlow != null)
            {
                earthGlowBaseScale = earthGlow.transform.localScale.x;
                if (earthGlow.sharedMaterial != null) earthGlowBase = earthGlow.sharedMaterial.GetFloat("_Intensity");
            }
            if (earth != null && earth.sharedMaterial != null)
            {
                earthEmissionBase = earth.sharedMaterial.GetFloat("_Emission");
                earthLightBase = ((Vector3)earth.sharedMaterial.GetVector("_LightDir")).normalized;
                earthAmbientBase = earth.sharedMaterial.GetFloat("_Ambient");
            }
            int n = lanterns?.Length ?? 0;
            lanternBaseEmission = new Color[n];
            for (int i = 0; i < n; i++)
            {
                var mats = lanterns[i] != null ? lanterns[i].sharedMaterials : null;
                lanternBaseEmission[i] = mats != null && mats.Length > lanternGlowMaterial && mats[lanternGlowMaterial] != null
                    ? mats[lanternGlowMaterial].GetColor("_EmissionColor") : Color.black;
            }
            lanternLightBase = new float[lanternLights?.Length ?? 0];
            for (int i = 0; i < lanternLightBase.Length; i++) lanternLightBase[i] = lanternLights[i] != null ? lanternLights[i].intensity : 0f;
            poolBase = new float[lanternPools?.Length ?? 0];
            for (int i = 0; i < poolBase.Length; i++)
                poolBase[i] = lanternPools[i] != null && lanternPools[i].sharedMaterial != null ? lanternPools[i].sharedMaterial.GetFloat("_Intensity") : 0f;
            if (cassiaTree != null && cassiaTree.sharedMaterials.Length > blossomMaterial && cassiaTree.sharedMaterials[blossomMaterial] != null)
                blossomBaseEmission = cassiaTree.sharedMaterials[blossomMaterial].GetColor("_EmissionColor");

            // One runtime copy of the halo material per instanced group (their colours differ); intensity set per frame.
            blossomHalo = HaloCopy("Blossom", blossomGlowColor);
            stoneHalo = HaloCopy("Stone", stoneGlowColor);
            orbHalo = HaloCopy("Orb", orbGlowColor);
            trailHalo = HaloCopy("Trail", trailColor);
            cityHalo = HaloCopy("EarthLights", earthLightColor);
            burstHalo = HaloCopy("Burst", trailColor);
            wishHalo = HaloCopy("Wish", wishColor);
            wishM = new Matrix4x4[wishCount];
            textHalo = HaloCopy("Text", Color.white);
            blossomM = new Matrix4x4[blossomPoints?.Length ?? 0];
            stoneM = new Matrix4x4[stonePoints?.Length ?? 0];
            trailM = new Matrix4x4[trailLength];
            trailPos = new Vector3[trailLength];
            trailTime = new float[trailLength];
            cityM = new Matrix4x4[earthLightCount];
            cityDir = new Vector3[earthLightCount];
            var rng = new System.Random(77);
            for (int i = 0; i < earthLightCount; i++)
            {
                // spread over the disc of the Earth seen from the garden, a little toward the night (left) side
                float a = (float)rng.NextDouble() * Mathf.PI * 2f, r = Mathf.Sqrt((float)rng.NextDouble()) * 0.85f;
                cityDir[i] = new Vector3(Mathf.Cos(a) * r - 0.15f, Mathf.Sin(a) * r, 0f);
            }

            progress.onComplete.AddListener(OnComplete);
            progress.onStrike.AddListener(_ => pulse = 1f);
            started = true;
        }

        Material HaloCopy(string n, Color c)
        {
            if (haloMaterial == null) return null;
            var m = new Material(haloMaterial) { name = "MR_" + n + "Halo (runtime)", enableInstancing = true };
            m.SetColor("_Color", c);
            return m;
        }

        void OnDisable()
        {
            if (!started) return;
            if (atmosphere != null) atmosphere.Apply();
            if (moonlight != null) moonlight.intensity = moonlightBase;
            if (bamboo != null) bamboo.silver = 0f;
            ClearBlocks();
        }

        void OnDestroy()
        {
            foreach (var m in new[] { blossomHalo, stoneHalo, orbHalo, trailHalo, cityHalo, textHalo, burstHalo, wishHalo })
                if (m != null) Destroy(m);
        }

        void Update()
        {
            if (!storyEnabled) return;
            float target = InEnding ? 1f : progress.Value;
            ShownProgress = Mathf.SmoothDamp(ShownProgress, target, ref velocity, smoothTime, Mathf.Infinity, Time.deltaTime);
            if (Mathf.Abs(ShownProgress - target) < 1e-4f) ShownProgress = target;
            pulse = Mathf.MoveTowards(pulse, 0f, Time.deltaTime * 2f);
            Apply(ShownProgress);
            DrawTrail();
            DrawEarthLights();
            DrawWishes();
        }

        static float Ramp(float v, float a, float b) { float t = Mathf.Clamp01((v - a) / Mathf.Max(1e-4f, b - a)); return t * t * (3f - 2f * t); }

        void Apply(float v)
        {
            float sv = Ramp(v, 0f, 1f);
            float mv = moonFollowsProgress ? v : moonFull;
            float msv = Ramp(mv, 0f, 1f);
            float p = pulse * pulse * strikePulse;

            // ---- Moon phase and brightness
            if (moon != null)
            {
                var d = (moon.transform.position - transform.position).normalized;
                var right = Vector3.Cross(Vector3.up, d).normalized; // lit on the right: a waxing Moon
                float th = crescentAngle * Mathf.Pow(1f - Mathf.Clamp01(mv), 0.55f) * Mathf.Deg2Rad; // half Moon at 0.5
                var L = (-d * Mathf.Cos(th) + right * Mathf.Sin(th) + Vector3.up * 0.1f * (1f - Mathf.Sin(th))).normalized;
                moon.GetPropertyBlock(mpb);
                mpb.SetVector("_LightDir", L);
                mpb.SetFloat("_Emission", Mathf.Lerp(moonEmission.x, moonEmission.y, msv) + p);
                mpb.SetFloat("_Ambient", Mathf.Lerp(moonDarkSide.x, moonDarkSide.y, msv));
                moon.SetPropertyBlock(mpb);
            }
            if (moonGlow != null)
            {
                moonGlow.GetPropertyBlock(mpb);
                mpb.SetFloat("_Intensity", Mathf.Lerp(moonGlowIntensity.x, moonGlowIntensity.y, msv) + p * 1.2f);
                moonGlow.SetPropertyBlock(mpb);
                moonGlow.transform.localScale = Vector3.one * (Mathf.Lerp(moonGlowScale.x, moonGlowScale.y, msv) + p * 0.6f);
            }

            // ---- light, ambient, fog: the garden brightens with the Moon
            if (moonlight != null) moonlight.intensity = moonlightBase * Mathf.Lerp(moonlightScale.x, moonlightScale.y, msv);
            if (atmosphere != null)
            {
                float a = Mathf.Lerp(ambientScale.x, ambientScale.y, msv);
                RenderSettings.ambientSkyColor = atmosphere.ambientSky * a;
                RenderSettings.ambientEquatorColor = atmosphere.ambientEquator * a;
                RenderSettings.ambientGroundColor = atmosphere.ambientGround * a;
                float f = Mathf.Lerp(fogScale.x, fogScale.y, sv);
                RenderSettings.fogDensity = atmosphere.fogDensity * f;
                RenderSettings.fogEndDistance = atmosphere.fogEnd / Mathf.Max(0.1f, f);
            }

            // ---- Earth (dark during the intro, glowing after the mochi arrives)
            if (earth != null)
            {
                earth.GetPropertyBlock(mpb);
                // dark Earth = its night side faces the garden: swing the light behind it (thin bright rim only)
                var back = (earth.transform.position - transform.position).normalized;
                var lit = Vector3.Slerp(Vector3.Slerp(back, earthLightBase, 0.25f), earthLightBase, Mathf.SmoothStep(0f, 1f, EarthBrightness));
                mpb.SetVector("_LightDir", lit);
                mpb.SetFloat("_Emission", earthEmissionBase * Mathf.Lerp(0.25f, 1f, EarthBrightness) + EarthPulse * 0.25f);
                mpb.SetFloat("_Ambient", earthAmbientBase * Mathf.Lerp(0.3f, 1f, EarthBrightness) + EarthPulse * 0.08f);
                earth.SetPropertyBlock(mpb);
            }
            if (earthGlow != null)
            {
                earthGlow.GetPropertyBlock(mpb);
                mpb.SetFloat("_Intensity", earthGlowBase * Mathf.Lerp(0.35f, 1f, EarthBrightness) + EarthPulse * 2.2f);
                earthGlow.SetPropertyBlock(mpb);
                earthGlow.transform.localScale = Vector3.one * (earthGlowBaseScale + EarthPulse * 3.4f);
            }

            // ---- lanterns one by one
            for (int i = 0; i < (lanterns?.Length ?? 0); i++)
            {
                float on = Ramp(v, lanternStart + lanternStep * i, lanternStart + lanternStep * i + 0.08f);
                if (lanterns[i] != null)
                {
                    lanterns[i].GetPropertyBlock(mpb, lanternGlowMaterial);
                    mpb.SetColor("_EmissionColor", lanternBaseEmission[i] * Mathf.Lerp(lanternEmber, 1f, on));
                    lanterns[i].SetPropertyBlock(mpb, lanternGlowMaterial);
                }
                if (i < lanternLightBase.Length && lanternLights[i] != null)
                {
                    lanternLights[i].intensity = lanternLightBase[i] * on;
                    lanternLights[i].enabled = on > 0.01f;
                }
                if (i < poolBase.Length && lanternPools[i] != null)
                {
                    lanternPools[i].GetPropertyBlock(mpb);
                    mpb.SetFloat("_Intensity", poolBase[i] * on);
                    lanternPools[i].SetPropertyBlock(mpb);
                    lanternPools[i].enabled = on > 0.01f;
                }
            }

            // ---- cassia blossoms
            float bloom = Ramp(v, blossomStart, 1f);
            if (cassiaTree != null && cassiaTree.sharedMaterials.Length > blossomMaterial)
            {
                cassiaTree.GetPropertyBlock(mpb, blossomMaterial);
                mpb.SetColor("_EmissionColor", blossomBaseEmission * Mathf.Lerp(blossomEmission.x, blossomEmission.y, bloom));
                cassiaTree.SetPropertyBlock(mpb, blossomMaterial);
            }
            var cam = MainCamera();
            int nb = blossomM.Length, kb = 0;
            for (int i = 0; i < nb; i++)
            {
                float a0 = blossomStart + (1f - blossomStart) * 0.75f * i / Mathf.Max(1, nb);
                float s = Ramp(v, a0, a0 + 0.15f);
                if (s <= 0.01f) continue;
                var pt = transform.TransformPoint(blossomPoints[i]);
                var rot = cam != null ? Quaternion.LookRotation(pt - cam.transform.position) : Quaternion.identity;
                float tw = 1f + 0.08f * Mathf.Sin(Time.time * 1.7f + i * 2.1f);
                blossomM[kb++] = Matrix4x4.TRS(pt, rot, Vector3.one * blossomHaloSize * s * tw);
            }
            DrawHalos(blossomHalo, blossomM, kb, Mathf.Lerp(0.25f, 0.9f, bloom));

            // ---- stepping stones toward the sky
            int ns = stoneM.Length, ks = 0;
            for (int i = 0; i < ns; i++)
            {
                float a0 = stoneStart + (0.97f - stoneStart) * i / Mathf.Max(1, ns);
                float s = Ramp(v, a0, a0 + 0.06f);
                if (s <= 0.01f) continue;
                var pt = transform.TransformPoint(stonePoints[i]) + Vector3.up * 0.06f;
                float pl = 1f + 0.06f * Mathf.Sin(Time.time * 1.3f - i * 0.6f);
                stoneM[ks++] = Matrix4x4.TRS(pt, Quaternion.Euler(90f, 0f, 0f), Vector3.one * stoneHaloSize * s * pl);
            }
            DrawHalos(stoneHalo, stoneM, ks, 0.55f);

            if (bamboo != null)
            {
                bamboo.silverColor = bambooSilver;
                bamboo.silver = bambooSilverAmount * Ramp(v, 0.9f, 1f);
            }
        }

        public void DrawHalos(Material m, Matrix4x4[] mats, int count, float intensity)
        {
            if (m == null || quadMesh == null || count <= 0) return;
            m.SetFloat("_Intensity", intensity);
            var rp = new RenderParams(m)
            {
                layer = gameObject.layer,
                worldBounds = new Bounds(transform.position, Vector3.one * 400f),
                shadowCastingMode = ShadowCastingMode.Off,
                receiveShadows = false,
            };
            Graphics.RenderMeshInstanced(rp, quadMesh, 0, mats, count);
        }

        void ClearBlocks()
        {
            if (moon != null) moon.SetPropertyBlock(null);
            if (moonGlow != null) { moonGlow.SetPropertyBlock(null); moonGlow.transform.localScale = Vector3.one * moonGlowBaseScale; }
            if (earth != null) earth.SetPropertyBlock(null);
            if (earthGlow != null) { earthGlow.SetPropertyBlock(null); earthGlow.transform.localScale = Vector3.one * earthGlowBaseScale; }
            if (lanterns != null) foreach (var l in lanterns) if (l != null) l.SetPropertyBlock(null, lanternGlowMaterial);
            if (lanternPools != null) foreach (var p in lanternPools) if (p != null) { p.SetPropertyBlock(null); p.enabled = true; }
            if (cassiaTree != null) cassiaTree.SetPropertyBlock(null, blossomMaterial);
            for (int i = 0; i < (lanternLights?.Length ?? 0); i++)
                if (lanternLights[i] != null) { lanternLights[i].enabled = true; lanternLights[i].intensity = lanternLightBase[i]; }
        }

        public static Camera MainCamera()
        {
            var c = Camera.main;
            return c != null ? c : FindAnyObjectByType<Camera>();
        }

        // ------------------------------------------------------------------ trail and Earth lights

        void PushTrail(Vector3 p)
        {
            trailPos[trailHead] = p;
            trailTime[trailHead] = Time.time;
            trailHead = (trailHead + 1) % trailLength;
            trailCount = Mathf.Min(trailCount + 1, trailLength);
        }

        void DrawTrail()
        {
            if (trailCount == 0 || Time.time > trailUntil) return;
            var cam = MainCamera();
            if (cam == null) return;
            int k = 0;
            for (int i = 0; i < trailCount; i++)
            {
                int idx = (trailHead - 1 - i + trailLength) % trailLength;
                float age = Time.time - trailTime[idx];
                float life = Mathf.Clamp01(1f - age / 0.9f);
                if (life <= 0f) continue;
                var p = trailPos[idx];
                float dist = Vector3.Distance(cam.transform.position, p);
                float tw = 0.75f + 0.25f * Mathf.Sin(Time.time * 23f + idx * 1.7f); // sparkle
                float size = Mathf.Max(0.05f, dist * 0.012f) * life * tw;
                trailM[k++] = Matrix4x4.TRS(p, Quaternion.LookRotation(p - cam.transform.position), Vector3.one * size);
            }
            DrawHalos(trailHalo, trailM, k, 0.8f);
        }

        void DrawEarthLights()
        {
            if ((cityOn <= 0.001f && Time.time - burstStart > 1.6f) || earth == null || earthLightCount == 0) return;
            var cam = MainCamera();
            if (cam == null) return;
            var c = earth.transform.position;
            float R = earth.transform.lossyScale.x;
            var toCam = (cam.transform.position - c).normalized;
            var right = Vector3.Cross(Vector3.up, toCam).normalized;
            var up = Vector3.Cross(toCam, right);
            int k = 0;
            for (int i = 0; i < earthLightCount; i++)
            {
                float on = Mathf.Clamp01(cityOn * earthLightCount * 1.3f - i); // one after another
                if (on <= 0.01f) continue;
                var d2 = cityDir[i];
                float z = Mathf.Sqrt(Mathf.Max(0f, 1f - d2.x * d2.x - d2.y * d2.y));
                var p = c + (-right * d2.x + up * d2.y + toCam * z) * R * 1.04f;
                float tw = 0.8f + 0.2f * Mathf.Sin(Time.time * 3.1f + i * 2.3f);
                cityM[k++] = Matrix4x4.TRS(p, Quaternion.LookRotation(p - cam.transform.position), Vector3.one * R * 0.6f * on * tw);
            }
            DrawHalos(cityHalo, cityM, k, 1.7f);

            // arrival burst: sparkles fly out from the Earth and fade (1.6 s)
            float age = Time.time - burstStart;
            if (age < 1.6f)
            {
                int kb = 0;
                float life = 1f - age / 1.6f;
                for (int i = 0; i < burstM.Length; i++)
                {
                    float ang = i * 2.399963f; // golden angle
                    float rr = R * (1.1f + 2.6f * Mathf.Sqrt(age / 1.6f) * (0.6f + 0.4f * ((i * 37) % 11) / 10f));
                    var pb = c + (right * Mathf.Cos(ang) + up * Mathf.Sin(ang)) * rr + toCam * R;
                    float tw = 0.7f + 0.3f * Mathf.Sin(Time.time * 19f + i * 1.3f);
                    burstM[kb++] = Matrix4x4.TRS(pb, Quaternion.LookRotation(pb - cam.transform.position), Vector3.one * R * 0.8f * life * tw);
                }
                DrawHalos(burstHalo, burstM, kb, 1.2f * life); // own material: intensity is read at render time
            }
        }

        void DrawWishes()
        {
            float t = Time.time - wishStart;
            if (t < 0f || t > wishRiseTime + 2.5f || earth == null || wishCount == 0) return;
            var cam = MainCamera();
            if (cam == null) return;
            var e = earth.transform.position;
            float R = earth.transform.lossyScale.x;
            var m = moon != null ? moon.transform.position : e + Vector3.up * 40f;
            var toCam = (cam.transform.position - e).normalized;
            var right = Vector3.Cross(Vector3.up, toCam).normalized;
            var up = Vector3.Cross(toCam, right);
            int k = 0;
            for (int i = 0; i < wishCount; i++)
            {
                // each wish leaves a point on the Earth's disc a little later than the previous one
                float ti = t - 1.6f * i / wishCount;
                if (ti <= 0f) continue;
                float u = Mathf.Clamp01(ti / wishRiseTime);
                float ang = i * 2.399963f, rr = Mathf.Sqrt((i + 0.5f) / wishCount) * 0.9f;
                var p0 = e + (right * Mathf.Cos(ang) + up * Mathf.Sin(ang)) * R * rr + toCam * R;
                // drift up and toward the Moon, spreading out like lanterns on the wind
                var goal = Vector3.Lerp(e, m, 0.55f) + Vector3.up * (18f + 10f * Mathf.Sin(i * 1.3f)) + right * ((i % 7) - 3f) * 4f;
                float ease = 1f - Mathf.Pow(1f - u, 2f);
                var p = Vector3.Lerp(p0, goal, ease) + right * Mathf.Sin(ti * 1.1f + i) * 0.8f;
                float d = Vector3.Distance(cam.transform.position, p);
                float fade = Mathf.Clamp01(ti / 0.6f) * Mathf.Clamp01((wishRiseTime + 2.5f - ti - 1.6f * i / wishCount) / 1.5f);
                float tw = 0.75f + 0.25f * Mathf.Sin(Time.time * 4.1f + i * 2.7f);
                wishM[k++] = Matrix4x4.TRS(p, Quaternion.LookRotation(p - cam.transform.position), Vector3.one * d * 0.026f * fade * tw);
            }
            DrawHalos(wishHalo, wishM, k, 2.0f);
        }

        // ------------------------------------------------------------------ ending

        void OnComplete()
        {
            if (!InEnding && isActiveAndEnabled && storyEnabled) StartCoroutine(Ending());
        }

        IEnumerator Ending()
        {
            InEnding = true;
            progress.Locked = true;
            float t0 = Time.time;

            // let the garden finish brightening while the mochi gets ready
            while (ShownProgress < 0.97f && Time.time - t0 < 2f) yield return null;

            Vector3 start = link.HasDough ? link.Centroid + Vector3.up * 0.02f
                          : link.Usu != null ? link.Usu.position + Vector3.up * (link.UsuBase + 0.06f)
                          : transform.position + Vector3.up * 1f;
            EnsureOrb();
            orb.SetActive(true);
            orbRenderer.SetPropertyBlock(null);
            link.SetDoughVisible(false);
            var orbT = orb.transform;
            var halo = orbT.GetChild(0);
            var target = earth != null ? earth.transform.position : transform.position + new Vector3(-80f, 85f, 100f);
            trailCount = 0;
            trailUntil = float.MaxValue;

            // rise out of the usu
            float t = 0f;
            Vector3 risen = start + Vector3.up * 0.5f;
            while (t < riseTime)
            {
                t += Time.deltaTime;
                float u = Mathf.Clamp01(t / riseTime);
                float e = 1f - (1f - u) * (1f - u);
                orbT.position = Vector3.Lerp(start, risen, e) + Vector3.up * 0.015f * Mathf.Sin(u * Mathf.PI * 2f);
                OrbLook(orbT, halo, Mathf.Lerp(0.75f, 1f, e), 1f);
                PushTrail(orbT.position);
                yield return null;
            }

            // a long curve out over the garden and up to the Earth
            var to = target - risen;
            var flat = new Vector3(to.x, 0f, to.z).normalized;
            var c1 = risen + Vector3.up * 2.2f + flat * 4f;
            var c2 = target - to * 0.4f - Vector3.up * 15f;
            t = 0f;
            while (t < flyTime)
            {
                t += Time.deltaTime;
                float u = Mathf.Clamp01(t / flyTime);
                float e = u * u * (3f - 2f * u);
                orbT.position = Bezier(risen, c1, c2, target, e);
                OrbLook(orbT, halo, 1f, Mathf.Lerp(1f, 0.3f, e * e * e));
                orbRenderer.GetPropertyBlock(mpb);
                mpb.SetFloat("_Glow", Mathf.Lerp(0.06f, 0.8f, e * e)); // melts into light as it reaches the Earth
                orbRenderer.SetPropertyBlock(mpb);
                PushTrail(orbT.position);
                yield return null;
            }
            orb.SetActive(false);
            trailUntil = Time.time + 1f;

            // the Earth glows and its lights come on
            progress.onMochiArrivedEarth.Invoke();
            burstStart = Time.time;
            if (!moonFollowsProgress) StartCoroutine(Fade(x => moonFull = x, 0f, 1f, 1.2f));
            StartCoroutine(EarthGlowPulse());
            if (afterEarthLit != null) yield return afterEarthLit();
            else
            {
                // wishes: warm lights rise from the Earth toward the Moon; the text follows once they are on their way
                wishStart = Time.time + 0.8f;
                yield return new WaitForSeconds(3.2f);
            }

            // text
            var cam = MainCamera();
            if (cam != null && !string.IsNullOrEmpty(endingText))
            {
                if (text == null) text = StoryText.Create(transform, "MR_EndingText", textHeight, textColor, textHalo, quadMesh);
                text.SetText(endingText);
                text.PlaceInFront(cam.transform, textDistance, textAboveEyes);
            }
            float shown = Time.time;
            float hold = Mathf.Max(3f, endingDuration - (shown - t0));
            while (Time.time - shown < hold)
            {
                float a = Time.time - shown;
                if (text != null) text.SetAlpha(Mathf.Clamp01(a / 1.2f) * Mathf.Clamp01((hold - a) / 1f));
                yield return null;
            }
            if (text != null) text.SetAlpha(0f);

            // reset for the next demo
            link.ResetDough();
            link.SetDoughVisible(true);
            InEnding = false;
            progress.ResetProgress();
            StartCoroutine(Fade(x => cityOn = x, cityOn, 0f, 1.5f));
            StartCoroutine(Fade(x => EarthPulse = x, EarthPulse, 0f, 1.5f));
            if (!moonFollowsProgress) StartCoroutine(Fade(x => moonFull = x, moonFull, 0f, 1.5f));
            Resetted?.Invoke();
        }

        IEnumerator EarthGlowPulse()
        {
            // soft swell (no white flash), settles to a warm glow; the lights come on one by one
            float b0 = EarthBrightness;
            for (float a = 0f; a < 0.6f; a += Time.deltaTime)
            {
                EarthPulse = Mathf.SmoothStep(0f, 1f, a / 0.6f);
                EarthBrightness = Mathf.Lerp(b0, 1f, a / 0.6f); // the dark Earth wakes up
                yield return null;
            }
            EarthBrightness = 1f;
            float t = 0f;
            while (InEnding)
            {
                t += Time.deltaTime;
                EarthPulse = Mathf.Lerp(0.45f, 1f, Mathf.Exp(-t * 0.9f)) + 0.05f * Mathf.Sin(t * 2.2f);
                cityOn = Mathf.Clamp01(t / 2.5f);
                yield return null;
            }
        }

        static IEnumerator Fade(System.Action<float> set, float from, float to, float time)
        {
            for (float a = 0f; a < time; a += Time.deltaTime) { set(Mathf.SmoothStep(from, to, a / time)); yield return null; }
            set(to);
        }

        void OrbLook(Transform orbT, Transform halo, float grow, float shrink)
        {
            var cam = MainCamera();
            float dist = cam != null ? Vector3.Distance(cam.transform.position, orbT.position) : 1f;
            float r = Mathf.Max(orbRadius * grow, dist * orbMinAngularRadius) * shrink;
            orbT.localScale = Vector3.one * r * 2f;
            if (cam != null) halo.rotation = Quaternion.LookRotation(halo.position - cam.transform.position);
        }

        static Vector3 Bezier(Vector3 a, Vector3 b, Vector3 c, Vector3 d, float t)
        {
            float u = 1f - t;
            return u * u * u * a + 3f * u * u * t * b + 3f * u * t * t * c + t * t * t * d;
        }

        void EnsureOrb()
        {
            if (orb != null) return;
            orb = new GameObject("MR_MochiOrb");
            orb.transform.SetParent(transform, true);
            orb.AddComponent<MeshFilter>().sharedMesh = sphereMesh;
            var mr = orb.AddComponent<MeshRenderer>();
            orbRenderer = mr;
            mr.sharedMaterial = orbMaterial;
            mr.shadowCastingMode = ShadowCastingMode.Off;
            mr.receiveShadows = false;
            mr.lightProbeUsage = LightProbeUsage.Off;
            var h = new GameObject("Glow");
            h.transform.SetParent(orb.transform, false);
            h.transform.localScale = Vector3.one * 2.2f;
            h.AddComponent<MeshFilter>().sharedMesh = quadMesh;
            var hr = h.AddComponent<MeshRenderer>();
            hr.sharedMaterial = orbHalo;
            hr.shadowCastingMode = ShadowCastingMode.Off;
            hr.receiveShadows = false;
            if (orbHalo != null) orbHalo.SetFloat("_Intensity", 0.18f);
            orb.SetActive(false);
        }
    }
}
