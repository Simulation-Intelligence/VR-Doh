using UnityEngine;
using UnityEngine.Rendering;

namespace MoonRabbit.World
{
    /// <summary>
    /// Rice to mochi, layered on the team's dough (its material is untouched). At progress 0 the dough is a heap of
    /// steamed rice: about a thousand grains (one instanced draw) packed over the whole surface, overlapping,
    /// every which way, half sticking out. Pounding melts them in: grains flatten along the surface normal, spread
    /// along it, lie down, take the dough's colour and sink until, from about 85 %, the surface is smooth mochi.
    /// A strike flattens the grains around the impact (6 cm) a step further at once.
    /// Each grain rides on one dough particle that was on the surface when the heap was made (particle readback via
    /// MochiLink), so grains move and fold with the dough instead of floating.
    /// </summary>
    [RequireComponent(typeof(MochiLink))]
    public class RiceToMochi : MonoBehaviour
    {
        public Material grainMaterial;
        [Range(0, 3000)] public int grainCount = 2400;
        [Tooltip("Grain length and width (m): steamed glutinous rice is about 7-9 x 4 mm.")]
        public Vector2 grainSize = new Vector2(0.0105f, 0.005f);
        [Tooltip("Grain centre outside the outermost particle at progress 0 (m), min/max. The dough surface is about 5 mm out.")]
        public Vector2 outward = new Vector2(0.003f, 0.0065f);
        [Tooltip("Strike: extra melt for grains within this radius of the kine face (m).")]
        public float strikeRadius = 0.06f;
        public float strikeMelt = 0.3f;
        public string warmLightName = "MR_MochiWarmLight";
        [Tooltip("A grain shows only where its particle has at least this many particles within ~2 cm (else it is on a crumb).")]
        public int minDensity = 14;
        [Tooltip("Brightness of the grains' warm-lit tint (1 = matches the dough under the warm light).")]
        public float warmScale = 1f;
        [Tooltip("Random sideways offset of a grain from its particle (m), so the heap is denser than the surface particles.")]
        public float tangentJitter = 0.006f;

        MochiLink link;
        Mesh mesh;
        int[] particle;
        Vector3[] pos, normal, hopOffset, hopVel;
        float[] yaw, tilt, offset, sizeVar, local, order, attached;
        Vector2[] jitter;
        Matrix4x4[] matrices;
        MaterialPropertyBlock mpb;
        Light warm;
        bool assigned, needAssign = true, hooked;
        int densityVersion = -1;
        float shownProgress;

        void Awake()
        {
            link = GetComponent<MochiLink>();
            mesh = BuildGrainMesh();
            mpb = new MaterialPropertyBlock();
        }

        void Hook(MoonProgress mp)
        {
            if (hooked) return;
            hooked = true;
            mp.onStrike.AddListener(Strike);
            mp.onReset.AddListener(() => needAssign = true);
        }

        static float Ramp(float v, float a, float b) { float t = Mathf.Clamp01((v - a) / (b - a)); return t * t * (3f - 2f * t); }

        void Update()
        {
            var mp = MoonProgress.Instance;
            if (mp != null) Hook(mp);
            float target = mp != null ? mp.Value : 0f;
            // eased like the rest of the story (melting never jumps)
            shownProgress = Mathf.MoveTowards(shownProgress, target, Time.deltaTime * (target < shownProgress ? 2f : 0.5f));

            if (!link.HasDough || grainMaterial == null || grainCount <= 0) return;
            if (needAssign || !assigned || particle.Length != grainCount) Assign();
            if (!assigned) return;

            float dt = Time.deltaTime;
            var P = link.Particles;
            bool densityCheck = link.Version != densityVersion;
            densityVersion = link.Version;
            var c = link.Centroid;
            float follow = 1f - Mathf.Exp(-dt * 40f);
            for (int i = 0; i < grainCount; i++)
            {
                int pi = particle[i];
                if (pi >= P.Length) continue;
                var p = P[pi];
                var n = p - c;
                n.y = n.y * 1.6f + 0.02f;
                n = n.sqrMagnitude > 1e-8f ? n.normalized : Vector3.up;
                normal[i] = pos[i] == Vector3.zero ? n : Vector3.Slerp(normal[i], n, follow);
                pos[i] = pos[i] == Vector3.zero ? p : Vector3.Lerp(pos[i], p, follow);
                local[i] = Mathf.Max(0f, local[i] - dt * 0.04f);
                // grains on torn-off crumbs (few particles around) fade away instead of floating in the air
                if (densityCheck) attached[i] = Mathf.MoveTowards(attached[i], link.DensityAt(p) >= minDensity ? 1f : 0f, dt * 6f);
                if (hopVel[i] != Vector3.zero)
                {
                    hopVel[i] += Physics.gravity * dt;
                    hopOffset[i] += hopVel[i] * dt;
                    if (hopOffset[i].y <= 0f && hopVel[i].y < 0f) { hopOffset[i] = Vector3.zero; hopVel[i] = Vector3.zero; }
                }
            }
            if (link.DoughRenderer != null && !link.DoughRenderer.enabled) return;
            Draw();
        }

        void Draw()
        {
            float p = shownProgress;
            float g = Mathf.Pow(p, 0.85f);                    // global melt
            float bury = Ramp(p, 0.8f, 1f);                   // last stretch: sink fully into the dough
            float thinning = Ramp(p, 0.45f, 1f) * 0.35f;      // only a few grains merge away early; most melt in place
            if (matrices == null || matrices.Length != grainCount) matrices = new Matrix4x4[grainCount];
            int k = 0;
            for (int i = 0; i < grainCount; i++)
            {
                if (order[i] < thinning) continue;
                float m = Mathf.Clamp01(g + local[i]);
                float shrink = (1f - Ramp(p, 0.9f, 1f)) * attached[i];
                if (shrink <= 0.01f) continue;
                float out_ = Mathf.Lerp(offset[i], -0.0015f, Mathf.Pow(m, 1.3f)) - bury * 0.004f;
                var r = Quaternion.FromToRotation(Vector3.up, normal[i]) * Quaternion.Euler(tilt[i] * (1f - m), yaw[i], 0f);
                float L = grainSize.x * sizeVar[i] * (1f + 0.7f * m);
                float W = grainSize.y * sizeVar[i] * (1f + 0.6f * m);
                float H = grainSize.y * sizeVar[i] * 0.9f * (1f - 0.85f * m);
                var nn = normal[i];
                var t1 = Vector3.Cross(nn, Mathf.Abs(nn.y) < 0.95f ? Vector3.up : Vector3.right).normalized;
                var t2 = Vector3.Cross(nn, t1);
                var side = (t1 * jitter[i].x + t2 * jitter[i].y) * (1f - 0.5f * m);
                matrices[k++] = Matrix4x4.TRS(pos[i] + side + nn * out_ + hopOffset[i], r, new Vector3(W, H, L) * shrink);
            }
            if (k == 0) return;

            mpb.SetFloat("_Melt", Mathf.Clamp01(g * 1.1f));
            if (warm == null) { var w = GameObject.Find(warmLightName); if (w != null) warm = w.GetComponent<Light>(); }
            if (warm != null && warm.isActiveAndEnabled)
            {
                var wp = warm.transform.position;
                mpb.SetVector("_WarmPos", new Vector4(wp.x, wp.y, wp.z, warm.range));
                mpb.SetColor("_WarmColor", new Color(0f, 0f, 0f, warmScale));
            }
            else mpb.SetColor("_WarmColor", new Color(0f, 0f, 0f, 0.55f));
            var sun = RenderSettings.sun;
            if (sun != null && sun.isActiveAndEnabled)
            {
                mpb.SetVector("_MainDir", -sun.transform.forward);
                mpb.SetColor("_MainColor", sun.color.linear * sun.intensity);
            }
            mpb.SetColor("_AmbSky", RenderSettings.ambientSkyColor.linear);
            mpb.SetColor("_AmbEquator", RenderSettings.ambientEquatorColor.linear);
            mpb.SetColor("_AmbGround", RenderSettings.ambientGroundColor.linear);

            var rp = new RenderParams(grainMaterial)
            {
                layer = gameObject.layer,
                worldBounds = new Bounds(link.Centroid, Vector3.one * 0.8f),
                shadowCastingMode = ShadowCastingMode.Off,
                receiveShadows = false,
                matProps = mpb,
            };
            for (int start = 0; start < k; start += 1023) // 1023 instances per call in the Built-in pipeline
                Graphics.RenderMeshInstanced(rp, mesh, 0, matrices, Mathf.Min(1023, k - start), start);
        }

        /// <summary>
        /// Surface particles: particles binned by direction from the dough's centre (44 x 24 bins over the upper
        /// dough); the two outermost of each bin are surface points. Grains pick among them at random.
        /// </summary>
        void Assign()
        {
            var P = link.Particles;
            if (P == null || P.Length == 0) return;
            int n = grainCount;
            particle = new int[n]; pos = new Vector3[n]; normal = new Vector3[n];
            hopOffset = new Vector3[n]; hopVel = new Vector3[n];
            yaw = new float[n]; tilt = new float[n]; offset = new float[n]; sizeVar = new float[n]; local = new float[n]; order = new float[n];
            jitter = new Vector2[n];
            attached = new float[n];

            const int AZ = 56, EL = 30;
            var best = new int[AZ * EL * 2];
            var bestD = new float[AZ * EL * 2];
            for (int b = 0; b < best.Length; b++) { best[b] = -1; bestD[b] = -1f; }
            var c = link.Centroid;
            for (int j = 0; j < P.Length; j++)
            {
                var d = P[j] - c;
                float r = d.magnitude;
                if (r < 1e-5f) continue;
                float el = Mathf.Asin(Mathf.Clamp(d.y / r, -1f, 1f)) * Mathf.Rad2Deg; // -90..90
                if (el < -25f) continue;                                              // under the dough: hidden by the bowl
                int ei = Mathf.Clamp((int)((el + 25f) / 115f * EL), 0, EL - 1);
                int ai = Mathf.Clamp((int)((Mathf.Atan2(d.z, d.x) + Mathf.PI) / (2f * Mathf.PI) * AZ), 0, AZ - 1);
                int b0 = (ei * AZ + ai) * 2;
                if (r > bestD[b0]) { bestD[b0 + 1] = bestD[b0]; best[b0 + 1] = best[b0]; bestD[b0] = r; best[b0] = j; }
                else if (r > bestD[b0 + 1]) { bestD[b0 + 1] = r; best[b0 + 1] = j; }
            }
            var candidates = new System.Collections.Generic.List<int>();
            foreach (var b in best) if (b >= 0) candidates.Add(b);
            if (candidates.Count == 0) return;

            var rng = new System.Random(1234);
            for (int i = candidates.Count - 1; i > 0; i--) { int s = rng.Next(i + 1); (candidates[i], candidates[s]) = (candidates[s], candidates[i]); }
            for (int i = 0; i < n; i++)
            {
                particle[i] = candidates[i % candidates.Count];
                yaw[i] = (float)rng.NextDouble() * 360f;
                tilt[i] = ((float)rng.NextDouble() - 0.5f) * 70f;             // every which way
                offset[i] = Mathf.Lerp(outward.x, outward.y, (float)rng.NextDouble());
                sizeVar[i] = Mathf.Lerp(0.85f, 1.15f, (float)rng.NextDouble());
                order[i] = (float)rng.NextDouble();
                float ja = (float)rng.NextDouble() * Mathf.PI * 2f, jr = Mathf.Sqrt((float)rng.NextDouble()) * tangentJitter;
                jitter[i] = new Vector2(Mathf.Cos(ja), Mathf.Sin(ja)) * (i < candidates.Count ? jr * 0.4f : jr);
                attached[i] = 1f;
            }
            assigned = true;
            needAssign = false;
        }

        void Strike(Vector3 at)
        {
            if (!assigned) return;
            float r2 = strikeRadius * strikeRadius;
            int hops = 0;
            for (int i = 0; i < grainCount; i++)
            {
                var d = pos[i] - at; d.y *= 0.5f;
                float dd = d.sqrMagnitude;
                if (dd > r2) continue;
                local[i] = Mathf.Min(1f, local[i] + strikeMelt * (1f - Mathf.Sqrt(dd) / strikeRadius));
                if (hops < 3 && dd > r2 * 0.6f && Random.value < 0.15f) // a couple of grains at the rim jump
                {
                    hops++;
                    var outDir = new Vector3(d.x, 0f, d.z).normalized;
                    hopVel[i] = Vector3.up * Random.Range(0.4f, 0.7f) + outDir * Random.Range(0.05f, 0.15f);
                    hopOffset[i] = Vector3.up * 0.001f;
                }
            }
        }

        static Mesh BuildGrainMesh()
        {
            // Low-poly ellipsoid, unit size (scaled per instance; y = height along the dough normal, z = length).
            const int A = 7, L = 5;
            var v = new Vector3[A * (L - 1) + 2];
            var nrm = new Vector3[v.Length];
            v[0] = new Vector3(0, 0, -0.5f); nrm[0] = Vector3.back;
            int idx = 1;
            for (int l = 1; l < L; l++)
            {
                float t = (float)l / L * Mathf.PI;
                float z = -Mathf.Cos(t) * 0.5f, r = Mathf.Sin(t) * 0.5f;
                for (int a = 0; a < A; a++)
                {
                    float ang = a * Mathf.PI * 2f / A;
                    v[idx] = new Vector3(Mathf.Cos(ang) * r, Mathf.Sin(ang) * r, z);
                    nrm[idx] = v[idx].normalized;
                    idx++;
                }
            }
            v[idx] = new Vector3(0, 0, 0.5f); nrm[idx] = Vector3.forward;
            var tris = new System.Collections.Generic.List<int>();
            for (int a = 0; a < A; a++) tris.AddRange(new[] { 0, 1 + a, 1 + (a + 1) % A });
            for (int l = 0; l < L - 2; l++)
                for (int a = 0; a < A; a++)
                {
                    int i0 = 1 + l * A + a, i1 = 1 + l * A + (a + 1) % A, j0 = i0 + A, j1 = i1 + A;
                    tris.AddRange(new[] { i0, j0, j1, i0, j1, i1 });
                }
            int last = idx, ring = 1 + (L - 2) * A;
            for (int a = 0; a < A; a++) tris.AddRange(new[] { last, ring + (a + 1) % A, ring + a });
            for (int i = 0; i < tris.Count; i += 3) { int x = tris[i + 1]; tris[i + 1] = tris[i + 2]; tris[i + 2] = x; } // outward winding
            var m = new Mesh { name = "MR_RiceGrain", vertices = v, normals = nrm, triangles = tris.ToArray() };
            m.RecalculateBounds();
            return m;
        }
    }
}
