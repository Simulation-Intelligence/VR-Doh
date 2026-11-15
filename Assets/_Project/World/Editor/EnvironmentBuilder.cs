using System.Collections.Generic;
using MoonRabbit.World;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;

namespace MoonRabbit.WorldEditor
{
    /// <summary>
    /// Generates the moonlit clearing from primitives and procedural meshes and saves it
    /// as Assets/_Project/World/Prefabs/MR_Environment.prefab.
    /// </summary>
    public static class EnvironmentBuilder
    {
        const string G = MRAssets.WorldGenerated;
        public const string PrefabPath = MRAssets.WorldRoot + "/Prefabs/MR_Environment.prefab";

        // ------------------------------------------------------------------ entry

        public static GameObject BuildPrefab(MoonRabbitEnvironmentSettings s, Scene buildScene)
        {
            var rng = new Rng(s.seed);
            var root = new GameObject("MR_Environment");
            SceneManager.MoveGameObjectToScene(root, buildScene);
            var t = root.transform;

            var mats = new Mats(s);
            BuildGround(s, t, mats, rng);
            BuildBamboo(s, t, mats, rng);
            BuildTree(s, t, mats, rng);
            BuildLanterns(s, t, mats, rng);
            BuildStones(s, t, mats, rng);
            if (s.pedestal && MoonRabbitMenu.PropBuilder == null) BuildPedestal(s, t, mats, rng);
            var moonlight = BuildMoonlight(s, t);
            if (s.mochiWarmLight) BuildMochiLight(s, t);
            var sky = BuildSky(s, rng);
            BuildCelestial(s, t);

            var atm = root.AddComponent<EnvironmentAtmosphere>();
            atm.skybox = sky;
            atm.moonlight = moonlight;
            atm.fog = true;
            atm.fogMode = s.fogMode;
            atm.fogColor = s.floatingIsland ? Color.Lerp(s.horizonFog, s.navySky, 0.65f) : s.horizonFog; // island: fog meets the open sky
            atm.fogDensity = s.floatingIsland ? s.fogDensity * 0.7f : s.fogDensity; // island: see past the edge into the sky
            atm.fogStart = s.fogStart;
            atm.fogEnd = s.fogEnd;
            atm.ambientSky = s.ambientSky;
            atm.ambientEquator = s.ambientEquator;
            atm.ambientGround = s.ambientGround;

            if (s.storyProgression) BuildStory(s, t, atm, moonlight);

            var prefab = MRAssets.SavePrefab(root, PrefabPath);
            Object.DestroyImmediate(root);
            return prefab;
        }

        // ------------------------------------------------------------------ materials

        class Mats
        {
            public readonly Material clearing, ground, stalkNear, stalkFar, leafNear, leafFar,
                trunk, canopy, canopyGold, stone, lanternGlow, bark, woodCut, earth, litter;

            public Mats(MoonRabbitEnvironmentSettings s)
            {
                clearing = MRAssets.Lit(G, "MR_Clearing", Color.Lerp(s.moonlight, s.navySky, 0.32f), 0.2f);
                ground = MRAssets.Lit(G, "MR_Ground", s.darkGround, 0.05f);
                stalkNear = MRAssets.Lit(G, "MR_BambooStalk_Near", s.bambooLight, 0.3f);
                stalkFar = MRAssets.Lit(G, "MR_BambooStalk_Far", s.bambooDark, 0.2f);
                leafNear = MRAssets.Lit(G, "MR_BambooLeaf_Near", Color.Lerp(s.bambooLight, s.bambooDark, 0.45f), 0.1f);
                leafFar = MRAssets.Lit(G, "MR_BambooLeaf_Far", Mul(s.bambooDark, 0.85f), 0.05f);
                trunk = MRAssets.Lit(G, "MR_CassiaTrunk", s.trunk, 0.05f);
                canopy = MRAssets.Lit(G, "MR_CassiaCanopy", Color.Lerp(s.cassiaGold, s.bambooLight, 0.4f), 0.15f, Mul(Color.Lerp(s.cassiaGold, s.bambooLight, 0.4f), 0.22f));
                canopyGold = MRAssets.Lit(G, "MR_CassiaGold", s.cassiaGold, 0.2f, Mul(s.cassiaGold, 0.45f));
                stone = MRAssets.Lit(G, "MR_Stone", s.stone, 0.1f);
                lanternGlow = MRAssets.Lit(G, "MR_LanternGlow", s.lantern, 0f, Mul(s.lantern, 2.2f));
                earth = MRAssets.Lit(G, "MR_ForestEarth", Mul(s.darkGround, 0.85f), 0.04f);
                litter = MRAssets.Lit(G, "MR_LeafLitter", Mul(Color.Lerp(s.lightWood, s.bambooDark, 0.5f), 0.75f), 0.05f);
                if (s.pedestal && MoonRabbitMenu.PropBuilder == null)
                {
                    bark = MRAssets.Lit(G, "MR_PedestalBark", Color.Lerp(s.wood, s.trunk, 0.45f), 0.08f);
                    woodCut = MRAssets.Lit(G, "MR_PedestalTop", s.lightWood, 0.15f);
                }
            }
        }

        // ------------------------------------------------------------------ ground

        // Ground profile by distance from the centre: the clearing stays nearly flat (player, pedestal), the forest floor
        // outside rolls more and more so it reads as earth rather than a calm water surface. Bamboo uses the same height.
        static readonly float[] GroundRadiiOffsets = { 0f, 0.6f, 1.5f, 3f, 5f, 8f, 12f, 16f, 22f };
        static readonly float[] GroundLift = { 0f, -0.01f, 0.01f, 0.05f, 0.09f, 0.12f, 0.16f, 0.22f, 0.28f };
        static readonly float[] GroundBump = { 0.004f, 0.04f, 0.09f, 0.16f, 0.22f, 0.3f, 0.4f, 0.5f, 0.55f };

        public static float GroundHeight(MoonRabbitEnvironmentSettings s, float x, float z)
        {
            float R = s.clearingRadius;
            float r = Mathf.Sqrt(x * x + z * z);
            if (r <= R) return Mathf.Lerp(0.015f, 0f, r / R) + (Noise3(new Vector3(x, 0f, z), 0.9f, 3f) - 0.5f) * 0.008f;
            float d = r - R;
            int k = 0;
            while (k < GroundRadiiOffsets.Length - 2 && d > GroundRadiiOffsets[k + 1]) k++;
            float t = Mathf.Clamp01(Mathf.InverseLerp(GroundRadiiOffsets[k], GroundRadiiOffsets[k + 1], d));
            float lift = Mathf.Lerp(GroundLift[k], GroundLift[k + 1], t);
            float bump = Mathf.Lerp(GroundBump[k], GroundBump[k + 1], t);
            float noise = (Noise3(new Vector3(x, 0f, z), 0.22f, 11f) - 0.5f) * 2f + (Noise3(new Vector3(x, 0f, z), 0.8f, 5f) - 0.5f) * 0.6f;
            float h = lift + noise * bump;
            if (s.floatingIsland)
            {
                // island: the forest floor rolls on to the edge and rounds off slightly at the lip
                float edge = IslandEdge(s, x, z);
                h = Mathf.Lerp(h, h - 0.35f, Mathf.SmoothStep(0f, 1f, Mathf.InverseLerp(edge - 1.8f, edge, r)));
                return h;
            }
            if (r > s.groundRadius - 6f) h = Mathf.Lerp(h, -0.3f, Mathf.InverseLerp(s.groundRadius - 6f, s.groundRadius, r)); // rim sinks into the fog
            return h;
        }

        /// <summary>Island edge radius in the direction of (x, z): islandRadius with a slow irregular wobble.</summary>
        public static float IslandEdge(MoonRabbitEnvironmentSettings s, float x, float z)
        {
            var d = new Vector3(x, 0f, z);
            d = d.sqrMagnitude > 1e-6f ? d.normalized : Vector3.forward;
            float n = (Noise3(d * 2.2f, 1f, 23f) - 0.5f) * 2f + (Noise3(d * 5f, 1f, 31f) - 0.5f) * 0.6f;
            return s.islandRadius * (1f + s.islandEdgeNoise * n);
        }

        static void BuildGround(MoonRabbitEnvironmentSettings s, Transform root, Mats m, Rng rng)
        {
            var mb = new MeshBuilder(2);
            int n = Mathf.Max(36, s.clearingSides);
            float R = s.clearingRadius;
            var radii = new List<float> { 0f, R * 0.5f, R };
            foreach (var o in GroundRadiiOffsets) if (o > 0f) radii.Add(R + o);
            float last = radii[radii.Count - 1];
            float outer = s.floatingIsland ? s.islandRadius : s.groundRadius;
            radii.RemoveAll(rr => rr > outer - 0.5f);
            last = radii[radii.Count - 1];
            for (float rr = last + 4f; rr < outer - 1f; rr += 4f) radii.Add(rr);
            if (s.floatingIsland) { radii.Add(outer - 0.9f); radii.Add(outer - 0.3f); }
            radii.Add(Mathf.Max(last + 1f, outer));

            var rings = new Vector3[radii.Count][];
            for (int k = 0; k < radii.Count; k++)
            {
                rings[k] = new Vector3[n];
                float offset = (k % 2) * 0.5f;
                for (int i = 0; i < n; i++)
                {
                    float a = (i + offset) / n * Mathf.PI * 2f;
                    float rk = radii[k];
                    // island: the rings beyond the clearing follow the irregular edge
                    if (s.floatingIsland && rk > R) rk = Mathf.Lerp(rk, rk * IslandEdge(s, Mathf.Sin(a), Mathf.Cos(a)) / s.islandRadius, Mathf.InverseLerp(R, s.islandRadius, rk));
                    float x = Mathf.Sin(a) * rk, z = Mathf.Cos(a) * rk;
                    rings[k][i] = new Vector3(x, GroundHeight(s, x, z), z);
                }
            }

            for (int k = 0; k < radii.Count - 1; k++)
            {
                int sub = radii[k + 1] <= R + 0.001f ? 0 : 1;
                var a = rings[k];
                var b = rings[k + 1];
                for (int i = 0; i < n; i++)
                {
                    int j = (i + 1) % n;
                    if (k == 0)
                    {
                        mb.Tri(new Vector3(0f, GroundHeight(s, 0f, 0f), 0f), b[i], b[j], Vector3.up, sub);
                        continue;
                    }
                    // rings are offset by half a step: zig-zag triangles
                    if (k % 2 == 1)
                    {
                        mb.Tri(a[i], b[i], b[j], Vector3.up, sub);
                        mb.Tri(a[i], b[j], a[j], Vector3.up, sub);
                    }
                    else
                    {
                        mb.Tri(a[i], b[i], a[j], Vector3.up, sub);
                        mb.Tri(a[j], b[i], b[j], Vector3.up, sub);
                    }
                }
            }

            islandEdgeRing = rings[rings.Length - 1];
            var mesh = MRAssets.SaveMesh(mb.ToMesh("MR_Ground"), $"{G}/Meshes/MR_Ground.asset");
            var go = MeshObject("MR_Ground", root, mesh, new[] { m.clearing, m.ground }, ShadowCastingMode.Off);
            if (s.mochiWarmLight && s.warmLightExcludedLayer >= 0) go.layer = s.warmLightExcludedLayer; // see the setting
            var col = go.AddComponent<BoxCollider>();
            col.center = new Vector3(0f, -0.05f, 0f);
            float cr = s.floatingIsland ? s.islandRadius : s.groundRadius;
            col.size = new Vector3(cr * 2f, 0.1f, cr * 2f);
            GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic);
            if (s.floatingIsland) BuildIslandUnderside(s, root, m, rng);
        }

        static Vector3[] islandEdgeRing;

        // ------------------------------------------------------------------ floating island underside

        /// <summary>Low-poly rocky underside (roughly an inverted cone) from the ground's edge ring down to a tip, an earthy
        /// band at the top, plus a few small rocks floating further out below. Flat-shaded, no collider, no shadows.</summary>
        static void BuildIslandUnderside(MoonRabbitEnvironmentSettings s, Transform root, Mats m, Rng rng)
        {
            var edge = islandEdgeRing;
            if (edge == null || edge.Length < 3) return;
            int n = edge.Length;
            var rock = MRAssets.Lit(G, "MR_IslandRock", s.islandRock, 0.06f);
            var mb = new MeshBuilder(2); // 0 earth band, 1 rock
            float D = Mathf.Max(3f, s.islandDepth);
            float[] depth = { 0f, 0.55f, 1.6f, 3.2f, 5.0f, 7.0f, 9.0f, 1f };
            float[] scale = { 1f, 0.97f, 0.86f, 0.68f, 0.5f, 0.32f, 0.15f, 0f };
            depth[depth.Length - 1] = D;
            for (int k = 1; k < depth.Length - 1; k++) depth[k] *= D / 11f;
            var rings = new Vector3[depth.Length][];
            for (int k = 0; k < depth.Length; k++)
            {
                rings[k] = new Vector3[n];
                for (int i = 0; i < n; i++)
                {
                    var e = edge[i];
                    if (k == 0) { rings[k][i] = e; continue; }
                    var flat = new Vector3(e.x, 0f, e.z);
                    float jag = k < depth.Length - 1 ? (Noise3(flat * 0.35f + Vector3.up * k, 1f, 41f) - 0.5f) * 0.28f : 0f;
                    var p = flat * (scale[k] * (1f + jag)) + Vector3.up * (e.y - depth[k] + (rng.Value - 0.5f) * 0.6f * Mathf.Min(1f, k / 2f));
                    rings[k][i] = p;
                }
            }
            var tip = new Vector3(0f, -D - 1.5f, 0f);
            for (int k = 0; k < depth.Length - 1; k++)
                for (int i = 0; i < n; i++)
                {
                    int j = (i + 1) % n;
                    int sub = k == 0 ? 0 : 1;
                    var a = rings[k][i]; var b = rings[k][j];
                    var c = k + 1 == depth.Length - 1 ? tip : rings[k + 1][j];
                    var d = k + 1 == depth.Length - 1 ? tip : rings[k + 1][i];
                    var outward = (new Vector3(a.x + b.x, 0f, a.z + b.z)).normalized + Vector3.down * 0.6f;
                    mb.Tri(a, b, c, outward, sub);
                    if (k + 1 < depth.Length - 1) mb.Tri(a, c, d, outward, sub);
                }
            var mesh = MRAssets.SaveMesh(mb.ToMesh("MR_IslandUnderside"), $"{G}/Meshes/MR_IslandUnderside.asset");
            var go = MeshObject("MR_IslandUnderside", root, mesh, new[] { m.earth, rock }, ShadowCastingMode.Off, false);
            if (s.mochiWarmLight && s.warmLightExcludedLayer >= 0) go.layer = s.warmLightExcludedLayer;
            GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic);

            // small rocks drifting below and around the island
            if (s.floatingRocks > 0)
            {
                var rb = new MeshBuilder(1);
                for (int i = 0; i < s.floatingRocks; i++)
                {
                    float az = rng.Range(0f, 360f);
                    if (Mathf.Abs(Mathf.DeltaAngle(az, s.moonAzimuth)) < 25f) az += 50f; // keep the view to the Moon clear
                    float r = rng.Range(s.islandRadius + 6f, s.islandRadius + 26f);
                    float size = rng.Range(0.8f, 2.6f);
                    var c = MoonRabbitEnvironmentSettings.Azimuth(az) * r + Vector3.up * rng.Range(-14f, -3f);
                    rb.Blob(c, new Vector3(size, size * 0.8f, size), 1, 0.3f, rng.Raw, 0);
                    rb.Blob(c + Vector3.down * size * 0.7f, new Vector3(size * 0.6f, size * 0.9f, size * 0.6f), 1, 0.3f, rng.Raw, 0);
                }
                var rmesh = MRAssets.SaveMesh(rb.ToMesh("MR_FloatingRocks"), $"{G}/Meshes/MR_FloatingRocks.asset");
                var rgo = MeshObject("MR_FloatingRocks", root, rmesh, new[] { rock }, ShadowCastingMode.Off, false);
                GameObjectUtility.SetStaticEditorFlags(rgo, StaticEditorFlags.BatchingStatic);
            }

            // slow star mist around and below the edge (runtime, one instanced draw)
            if (s.skyMistPuffs > 0)
            {
                var glow = Shader.Find("MoonRabbit/Glow");
                if (glow != null)
                {
                    var mat = MRAssets.GetMaterial($"{G}/Materials/MR_SkyMist.mat", glow);
                    mat.SetTexture("_MainTex", HaloTexture());
                    mat.SetColor("_Color", s.skyMistColor);
                    mat.SetFloat("_Intensity", 0.3f);
                    mat.enableInstancing = true;
                    EditorUtility.SetDirty(mat);
                    var mist = new GameObject("MR_SkyMist");
                    mist.transform.SetParent(root, false);
                    var sm = mist.AddComponent<SkyMist>();
                    sm.material = mat;
                    sm.quad = Resources.GetBuiltinResource<Mesh>("Quad.fbx");
                    sm.count = s.skyMistPuffs;
                    sm.radius = new Vector2(s.islandRadius - 2f, s.islandRadius + 16f);
                    sm.height = new Vector2(-9f, -1.5f);
                }
            }
        }

        // ------------------------------------------------------------------ bamboo

        static Mesh BambooVariant(MoonRabbitEnvironmentSettings s, Rng rng, int index)
        {
            var mb = new MeshBuilder(2);
            float h = Mathf.Lerp(s.bambooHeight.x, s.bambooHeight.y, (index + 0.5f) / Mathf.Max(1, s.bambooVariants));
            float r = rng.Range(s.bambooStalkRadius);
            int segs = Mathf.Max(3, Mathf.RoundToInt(h / Mathf.Max(0.2f, s.bambooSegmentLength)));
            float bend = rng.Range(0.1f, 0.35f) * (rng.Value < 0.5f ? -1f : 1f);

            Vector3 P(float t) => new Vector3(bend * t * t, h * t, 0f);
            float Rad(float t) => r * Mathf.Lerp(1f, 0.6f, t);

            for (int k = 0; k < segs; k++)
            {
                float t0 = (float)k / segs, t1 = (float)(k + 1) / segs;
                var p0 = P(t0);
                var p1 = P(t1);
                var dir = (p1 - p0).normalized;
                mb.Frustum(p0, p1, Rad(t0), Rad(t1), 6, 0, false, false, k * 0.3f);
                if (k < segs - 1)
                    mb.Frustum(p1 - dir * 0.025f, p1 + dir * 0.025f, Rad(t1) * 1.2f, Rad(t1) * 1.2f, 6, 0, true, true, k * 0.3f);
                else
                    mb.Frustum(p1, p1 + dir * 0.04f, Rad(t1), 0f, 6, 0, false, false);
            }

            // a few leaf clusters on short twigs, upper half only
            int left = Mathf.Max(0, s.leavesPerStalk);
            while (left > 0)
            {
                float t = Mathf.Round(rng.Range(0.45f, 0.97f) * segs) / segs;
                var b = P(Mathf.Clamp(t, 1f / segs, 1f));
                float az = rng.Range(0f, Mathf.PI * 2f);
                var outDir = new Vector3(Mathf.Cos(az), 0f, Mathf.Sin(az));
                var twigDir = (outDir + Vector3.up * rng.Range(0.3f, 0.8f)).normalized;
                var tip = b + twigDir * rng.Range(0.25f, 0.45f);
                mb.Frustum(b, tip, 0.008f, 0.005f, 3, 0, false, false);

                int here = Mathf.Min(left, rng.Int(3, 6));
                left -= here;
                for (int l = 0; l < here; l++)
                {
                    float spread = rng.Range(-55f, 55f);
                    float droop = rng.Range(15f, 55f) * Mathf.Deg2Rad;
                    var horiz = Quaternion.AngleAxis(spread, Vector3.up) * outDir;
                    var ld = (horiz * Mathf.Cos(droop) + Vector3.down * Mathf.Sin(droop)).normalized;
                    float len = rng.Range(0.26f, 0.42f);
                    float w = len * 0.16f;
                    var side = Vector3.Cross(ld, Vector3.up);
                    side = side.sqrMagnitude < 1e-4f ? Vector3.right : side.normalized;
                    var mid = tip + ld * len * 0.4f;
                    var end = tip + ld * len + Vector3.down * len * 0.12f;
                    mb.TwoSided(tip, mid + side * w, end, 1);
                    mb.TwoSided(tip, end, mid - side * w, 1);
                }
            }
            return mb.ToMesh($"MR_Bamboo_{index}");
        }

        static void BuildBamboo(MoonRabbitEnvironmentSettings s, Transform root, Mats m, Rng rng)
        {
            int V = Mathf.Clamp(s.bambooVariants, 1, 8);
            var meshes = new Mesh[V];
            for (int v = 0; v < V; v++)
                meshes[v] = MRAssets.SaveMesh(BambooVariant(s, rng, v), $"{G}/Meshes/MR_Bamboo_{v}.asset");

            var near = new List<Matrix4x4>[V];
            var far = new List<Matrix4x4>[V];
            for (int v = 0; v < V; v++) { near[v] = new List<Matrix4x4>(); far[v] = new List<Matrix4x4>(); }

            float inner = s.bambooInnerRadius, outer = Mathf.Max(inner + 0.5f, s.bambooOuterRadius);
            float midR = (inner + outer) * 0.5f;
            var placed = new List<Vector3>();
            var clumps = new List<List<Vector3>>();
            int made = 0, guard = 0, skippedBack = 0;

            bool InOpening(float azDeg) => Mathf.Abs(Mathf.DeltaAngle(azDeg, s.openingDirection)) < s.openingWidth * 0.5f;

            while (made < s.bambooCount && guard++ < s.bambooCount * 60)
            {
                float az = rng.Range(0f, 360f);
                if (InOpening(az)) continue;
                float rr = Mathf.Lerp(inner, outer, Mathf.Pow(rng.Value, s.bambooInnerBias)); // >1: denser toward the clearing
                var clump = MoonRabbitEnvironmentSettings.Azimuth(az) * rr;
                int inClump = rng.Int(2, 6);
                var clumpStalks = new List<Vector3>();
                for (int j = 0; j < inClump && made < s.bambooCount; j++)
                {
                    var p = clump + new Vector3(rng.Range(-0.6f, 0.6f), 0f, rng.Range(-0.6f, 0.6f));
                    float pr = new Vector2(p.x, p.z).magnitude;
                    if (pr < inner || pr > outer + 0.5f) continue;
                    if (InOpening(Mathf.Atan2(p.x, p.z) * Mathf.Rad2Deg)) continue;
                    bool crowded = false;
                    foreach (var q in placed) if ((q - p).sqrMagnitude < 0.22f * 0.22f) { crowded = true; break; }
                    if (crowded) continue;
                    placed.Add(p);

                    var radial = new Vector3(p.x, 0f, p.z).normalized;
                    float lean = (s.outwardLean + rng.Range(-2f, 3f)) * Mathf.Deg2Rad;
                    var tilt = Quaternion.FromToRotation(Vector3.up, (Vector3.up + radial * Mathf.Tan(lean)).normalized);
                    var rot = tilt * Quaternion.Euler(0f, rng.Range(0f, 360f), 0f);
                    float sc = rng.Range(0.85f, 1.15f);
                    p.y = GroundHeight(s, p.x, p.z);
                    int vi = rng.Int(0, V);
                    // behind the player (away from the Moon): leave some stalks out; same draws, so the rest keeps its layout
                    float back = Mathf.Abs(Mathf.DeltaAngle(Mathf.Atan2(p.x, p.z) * Mathf.Rad2Deg, s.moonAzimuth + 180f));
                    float thin = s.bambooBackThinning * (1f - Mathf.SmoothStep(0f, 1f, Mathf.InverseLerp(60f, 95f, back)));
                    if (thin > 0f && Hash01(p * 1.7f + Vector3.one * 3.1f) < thin) { made++; skippedBack++; continue; }
                    clumpStalks.Add(p);
                    var scale = Vector3.one * sc;
                    if (s.earthViewClear)
                    {
                        // toward the Earth: younger, lower bamboo so the ending's flight stays in view; grows back smoothly
                        var eye = new Vector3(s.playerPosition.x, 0f, s.playerPosition.z);
                        var d = new Vector3(p.x, 0f, p.z) - eye;
                        float off = Mathf.Abs(Mathf.DeltaAngle(Mathf.Atan2(d.x, d.z) * Mathf.Rad2Deg, s.earthAzimuth));
                        float w = 1f - Mathf.SmoothStep(0f, 1f, Mathf.InverseLerp(s.earthViewHalfWidth, s.earthViewHalfWidth + s.earthViewFalloff, off));
                        if (w > 0f)
                        {
                            float h = Mathf.Lerp(s.bambooHeight.x, s.bambooHeight.y, (vi + 0.5f) / V) * sc;
                            float allowed = s.earthViewEyeHeight + d.magnitude * Mathf.Tan(s.earthViewMaxElevation * Mathf.Deg2Rad);
                            float k = Mathf.Lerp(1f, Mathf.Clamp(allowed / h, 0.25f, 1f) * Mathf.Lerp(0.85f, 1f, Hash01(p)), w); // no rng draw: the rest of the grove keeps its layout
                            scale = new Vector3(sc * Mathf.Sqrt(k), sc * k, sc * Mathf.Sqrt(k)); // shorter and a little slimmer
                        }
                    }
                    var mtx = Matrix4x4.TRS(p + Vector3.down * s.bambooSink, rot, scale);
                    (pr < midR ? near : far)[vi].Add(mtx);
                    made++;
                }
                if (clumpStalks.Count > 0) clumps.Add(clumpStalks);
            }
            BuildBambooBases(s, root, m, rng, clumps);
            BambooPlaced = made - skippedBack;

            var grove = new GameObject("MR_Bamboo");
            grove.transform.SetParent(root, false);

            if (s.useGpuInstancing)
            {
                var g = grove.AddComponent<BambooGrove>();
                g.batches = new List<BambooGrove.Batch>();
                for (int v = 0; v < V; v++)
                {
                    if (near[v].Count > 0)
                        g.batches.Add(new BambooGrove.Batch { mesh = meshes[v], materials = new[] { m.stalkNear, m.leafNear }, localMatrices = near[v].ToArray() });
                    if (far[v].Count > 0)
                        g.batches.Add(new BambooGrove.Batch { mesh = meshes[v], materials = new[] { m.stalkFar, m.leafFar }, localMatrices = far[v].ToArray() });
                }
                float hMax = s.bambooHeight.y * 1.2f + 1f;
                g.localBounds = new Bounds(new Vector3(0f, hMax * 0.5f, 0f), new Vector3((outer + 2f) * 2f, hMax, (outer + 2f) * 2f));
                g.castShadows = ShadowCastingMode.Off;
            }
            else
            {
                int i = 0;
                for (int v = 0; v < V; v++)
                {
                    foreach (var (list, stalk, leaf) in new[] { (near[v], m.stalkNear, m.leafNear), (far[v], m.stalkFar, m.leafFar) })
                        foreach (var mtx in list)
                        {
                            var go = MeshObject($"Bamboo_{i++:000}", grove.transform, meshes[v], new[] { stalk, leaf }, ShadowCastingMode.Off);
                            go.transform.localPosition = mtx.GetColumn(3);
                            go.transform.localRotation = mtx.rotation;
                            go.transform.localScale = mtx.lossyScale;
                            GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic);
                        }
                }
            }
        }

        /// <summary>Grounds the bamboo: a low earth mound under every clump (the stalks are sunk into it), dry leaf litter
        /// and a few pebbles around it. One static mesh, no shadows.</summary>
        static void BuildBambooBases(MoonRabbitEnvironmentSettings s, Transform root, Mats m, Rng rng, List<List<Vector3>> clumps)
        {
            var mb = new MeshBuilder(3); // 0 earth, 1 litter, 2 stone
            foreach (var clump in clumps)
            {
                var c = Vector3.zero;
                foreach (var p in clump) c += p;
                c /= clump.Count;
                float spread = 0f;
                foreach (var p in clump) spread = Mathf.Max(spread, new Vector2(p.x - c.x, p.z - c.z).magnitude);
                float rad = spread + rng.Range(0.3f, 0.45f);
                float gy = GroundHeight(s, c.x, c.z);
                mb.Blob(new Vector3(c.x, gy - 0.05f, c.z), new Vector3(rad, rng.Range(0.13f, 0.18f), rad * rng.Range(0.8f, 1f)), 1, 0.25f, rng.Raw, 0);

                int leaves = clump.Count * 5;
                for (int i = 0; i < leaves; i++)
                {
                    float a = rng.Range(0f, Mathf.PI * 2f);
                    float d = rad * rng.Range(0.75f, 1.6f);
                    float x = c.x + Mathf.Cos(a) * d, z = c.z + Mathf.Sin(a) * d;
                    var q = new Vector3(x, GroundHeight(s, x, z) + 0.012f, z);
                    float yaw = rng.Range(0f, 360f), len = rng.Range(0.14f, 0.24f);
                    var dir = Quaternion.Euler(0f, yaw, 0f) * Vector3.forward;
                    var side = Vector3.Cross(Vector3.up, dir) * len * 0.18f;
                    mb.TwoSided(q - dir * len * 0.5f, q + side + Vector3.up * 0.01f, q + dir * len * 0.5f, 1);
                    mb.TwoSided(q - dir * len * 0.5f, q + dir * len * 0.5f, q - side + Vector3.up * 0.01f, 1);
                }
                int pebbles = rng.Int(1, 4);
                for (int i = 0; i < pebbles; i++)
                {
                    float a = rng.Range(0f, Mathf.PI * 2f);
                    float d = rad * rng.Range(0.85f, 1.25f);
                    float x = c.x + Mathf.Cos(a) * d, z = c.z + Mathf.Sin(a) * d;
                    float sz = rng.Range(0.05f, 0.12f);
                    mb.Blob(new Vector3(x, GroundHeight(s, x, z) + sz * 0.15f, z), new Vector3(sz, sz * 0.55f, sz * rng.Range(0.7f, 1f)), 0, 0.2f, rng.Raw, 2);
                }
            }
            var mesh = MRAssets.SaveMesh(mb.ToMesh("MR_BambooBases"), $"{G}/Meshes/MR_BambooBases.asset");
            var go = MeshObject("MR_BambooBases", root, mesh, new[] { m.earth, m.litter, m.stone }, ShadowCastingMode.Off);
            if (s.mochiWarmLight && s.warmLightExcludedLayer >= 0) go.layer = s.warmLightExcludedLayer; // see the setting
            GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic);
        }

        // ------------------------------------------------------------------ cassia tree

        static void BuildTree(MoonRabbitEnvironmentSettings s, Transform root, Mats m, Rng rng)
        {
            var mb = new MeshBuilder(3);
            float H = s.treeHeight;
            float R = s.canopyRadius;

            Vector3 T(float t) => new Vector3(Mathf.Sin(t * 2.2f) * 0.18f, H * 0.62f * t, Mathf.Sin(t * 1.3f) * 0.08f);

            // trunk: one continuous tube (separate frustums left visible gaps at every bend)
            const int trunkRings = 9;
            var tp = new List<Vector3>(); var tr = new List<float>();
            for (int k = 0; k < trunkRings; k++)
            {
                float t = k / (trunkRings - 1f);
                tp.Add(T(t)); tr.Add(Mathf.Lerp(0.2f, 0.09f, t));
            }
            tp.Add(T(1f) + (T(1f) - T(0.9f)).normalized * 0.35f); tr.Add(0.07f); // runs on into the canopy
            mb.Tube(tp, tr, 8, 0, true, false);

            for (int i = 0; i < 5; i++) // root flare: starts inside the trunk, ends below the ground line
            {
                var d = MoonRabbitEnvironmentSettings.Azimuth(i * 72f + rng.Range(-15f, 15f));
                mb.Tube(new[] { d * 0.02f + Vector3.up * 0.38f, d * 0.2f + Vector3.up * 0.14f, d * 0.48f + Vector3.down * 0.03f },
                        new[] { 0.09f, 0.06f, 0.025f }, 5, 0, false, true);
            }

            var c = T(1f) + Vector3.up * R * 0.55f;
            for (int i = 0; i < 3; i++) // branches: start inside the trunk axis, curve up into the canopy
            {
                float tb = rng.Range(0.6f, 0.95f);
                var start = T(tb);
                var dir = (MoonRabbitEnvironmentSettings.Azimuth(i * 120f + rng.Range(-30f, 30f)) * 0.8f + Vector3.up).normalized;
                var end = start + dir * R * 0.75f;
                var mid = Vector3.Lerp(start, end, 0.5f) + Vector3.down * 0.06f;
                float r0 = Mathf.Lerp(0.2f, 0.09f, tb) * 0.75f;
                mb.Tube(new[] { start - dir * 0.04f, mid, end }, new[] { r0, Mathf.Lerp(r0, 0.035f, 0.5f), 0.035f }, 6, 0, false, true);
            }

            // round canopy: one big blob plus smaller ones around it
            mb.Blob(c, new Vector3(1f, 0.8f, 1f) * R * 0.78f, 1, 0.12f, rng.Raw, 1);
            for (int i = 0; i < Mathf.Max(0, s.canopyBlobs - 1); i++)
            {
                var d = RandomDir(rng, -0.3f, 0.8f);
                var pos = c + Vector3.Scale(d, new Vector3(R * 0.7f, R * 0.45f, R * 0.7f));
                float r = R * rng.Range(0.38f, 0.55f);
                mb.Blob(pos, new Vector3(r, r * 0.85f, r), 1, 0.15f, rng.Raw, i % 3 == 0 ? 2 : 1);
            }
            // golden cassia blossoms on the surface
            for (int i = 0; i < s.blossomCount; i++)
            {
                var d = RandomDir(rng, -0.2f, 1f);
                var p = c + Vector3.Scale(d, new Vector3(R * 1.05f, R * 0.85f, R * 1.05f));
                mb.Blob(p, Vector3.one * rng.Range(0.05f, 0.09f), 0, 0.1f, rng.Raw, 2);
                blossomLocal.Add(p);
            }

            var mesh = MRAssets.SaveMesh(mb.ToMesh("MR_CassiaTree"), $"{G}/Meshes/MR_CassiaTree.asset");
            var go = MeshObject("MR_CassiaTree", root, mesh, new[] { m.trunk, m.canopy, m.canopyGold }, ShadowCastingMode.On);
            go.transform.localPosition = s.treePosition;
            go.transform.localRotation = Quaternion.Euler(0f, rng.Range(0f, 360f), 0f);
            blossomPoints.Clear();
            foreach (var bp in blossomLocal) blossomPoints.Add(go.transform.localPosition + go.transform.localRotation * bp);
            blossomLocal.Clear();
            var col = go.AddComponent<CapsuleCollider>();
            col.center = new Vector3(0f, H * 0.31f, 0f);
            col.radius = 0.2f;
            col.height = H * 0.62f;
            GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic);
        }

        // ------------------------------------------------------------------ stone lanterns

        static void BuildLanterns(MoonRabbitEnvironmentSettings s, Transform root, Mats m, Rng rng)
        {
            var mb = new MeshBuilder(2);
            var up = Vector3.up;
            mb.Frustum(Vector3.zero, up * 0.12f, 0.26f, 0.22f, 6, 0);              // base
            mb.Frustum(up * 0.12f, up * 0.6f, 0.085f, 0.075f, 6, 0);               // post
            mb.Frustum(up * 0.6f, up * 0.7f, 0.2f, 0.24f, 6, 0);                   // platform
            for (int i = 0; i < 4; i++)                                             // corner pillars
            {
                var d = MoonRabbitEnvironmentSettings.Azimuth(45f + 90f * i) * 0.13f;
                mb.Frustum(d + up * 0.7f, d + up * 0.92f, 0.028f, 0.028f, 4, 0, false, false);
            }
            mb.Frustum(up * 0.72f, up * 0.9f, 0.1f, 0.1f, 4, 1, true, true, Mathf.PI / 4f); // glowing light box
            mb.Frustum(up * 0.92f, up * 1.08f, 0.36f, 0.06f, 6, 0);                // roof
            mb.Blob(up * 1.14f, Vector3.one * 0.06f, 0, 0.05f, rng.Raw, 0);         // finial

            var mesh = MRAssets.SaveMesh(mb.ToMesh("MR_StoneLantern"), $"{G}/Meshes/MR_StoneLantern.asset");
            var positions = s.lanternPositions ?? new Vector3[0];
            for (int i = 0; i < positions.Length; i++)
            {
                var go = MeshObject($"MR_StoneLantern_{i}", root, mesh, new[] { m.stone, m.lanternGlow }, ShadowCastingMode.On);
                go.transform.localPosition = positions[i];
                var toCenter = s.mortarPosition - positions[i];
                toCenter.y = 0f;
                if (toCenter.sqrMagnitude > 1e-4f) go.transform.localRotation = Quaternion.LookRotation(toCenter);
                var col = go.AddComponent<CapsuleCollider>();
                col.center = new Vector3(0f, 0.57f, 0f);
                col.radius = 0.25f;
                col.height = 1.15f;
                GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic);

                var lightGo = new GameObject("LanternLight");
                lightGo.transform.SetParent(go.transform, false);
                lightGo.transform.localPosition = up * 0.81f;
                var l = lightGo.AddComponent<Light>();
                l.type = LightType.Point;
                l.color = s.lantern;
                l.range = s.lanternLightRange;
                l.intensity = s.lanternLightIntensity;
                l.shadows = LightShadows.None; // the moonlight is the only real-time shadow
            }
            if (s.lanternGlowPools) BuildLanternPools(s, root, positions);
        }

        /// <summary>Soft additive discs on the ground under the lanterns: a fake light pool, no lighting cost.</summary>
        static void BuildLanternPools(MoonRabbitEnvironmentSettings s, Transform root, Vector3[] positions)
        {
            var glow = Shader.Find("MoonRabbit/Glow");
            if (glow == null) return;
            var mat = MRAssets.GetMaterial($"{G}/Materials/MR_LanternPool.mat", glow);
            mat.SetTexture("_MainTex", HaloTexture());
            mat.SetColor("_Color", s.lantern);
            mat.SetFloat("_Intensity", s.lanternPoolIntensity);
            mat.renderQueue = 2999; // before other transparents (it lies on the ground)
            EditorUtility.SetDirty(mat);
            var quad = Resources.GetBuiltinResource<Mesh>("Quad.fbx");
            for (int i = 0; i < positions.Length; i++)
            {
                var p = positions[i];
                var go = MeshObject($"MR_LanternPool_{i}", root, quad, new[] { mat }, ShadowCastingMode.Off, false);
                go.transform.localPosition = new Vector3(p.x, GroundHeight(s, p.x, p.z) + 0.02f, p.z);
                go.transform.localRotation = Quaternion.Euler(90f, 0f, 0f); // quad faces up
                go.transform.localScale = Vector3.one * (s.lanternPoolRadius * 2f);
            }
        }

        // ------------------------------------------------------------------ story progression

        static readonly List<Vector3> blossomLocal = new List<Vector3>(), blossomPoints = new List<Vector3>(), stonePoints = new List<Vector3>();

        /// <summary>MR_Story: MoonProgress + MochiLink + DemoStrikeDetector + RiceToMochi + MoonStory, wired to the
        /// environment's Moon, lanterns, tree, stones and bamboo. Everything it changes is runtime only.</summary>
        static void BuildStory(MoonRabbitEnvironmentSettings s, Transform root, EnvironmentAtmosphere atm, Light moonlight)
        {
            var go = new GameObject("MR_Story");
            go.transform.SetParent(root, false);
            var progress = go.AddComponent<MoonProgress>();
            progress.demoStrikesToFull = Mathf.Max(1, s.demoStrikesToFull);
            go.AddComponent<MochiLink>();
            go.AddComponent<DemoStrikeDetector>();

            var rice = go.AddComponent<RiceToMochi>();
            rice.grainCount = s.riceGrainCount;
            var riceShader = Shader.Find("MoonRabbit/RiceGrain");
            if (riceShader != null)
            {
                var rm = MRAssets.GetMaterial($"{G}/Materials/MR_RiceGrain.mat", riceShader);
                rm.shader = riceShader;
                rm.SetColor("_Color", new Color(0.95f, 0.94f, 0.91f));
                rm.SetColor("_DoughColor", new Color(0.965f, 0.955f, 0.925f)); // MochiSurface.mat _Color
                rm.SetColor("_Lit", new Color(0.95f, 0.8f, 0.71f)); // how the warm-lit dough reads
                rm.SetFloat("_Sheen", 0.35f);
                rm.SetFloat("_Translucency", 0.35f);
                rm.enableInstancing = true;
                EditorUtility.SetDirty(rm);
                rice.grainMaterial = rm;
            }
            else Debug.LogError("[Moon Rabbit] MoonRabbit/RiceGrain shader missing (Assets/_Project/World/Shaders).");

            var story = go.AddComponent<MoonStory>();
            var glow = Shader.Find("MoonRabbit/Glow");
            if (glow != null)
            {
                var halo = MRAssets.GetMaterial($"{G}/Materials/MR_StoryHalo.mat", glow);
                halo.SetTexture("_MainTex", HaloTexture());
                halo.SetColor("_Color", Color.white);
                halo.SetFloat("_Intensity", 1f);
                halo.enableInstancing = true;
                EditorUtility.SetDirty(halo);
                story.haloMaterial = halo;
            }
            var orbShader = Shader.Find("MoonRabbit/MochiOrb");
            if (orbShader != null)
            {
                var orb = MRAssets.GetMaterial($"{G}/Materials/MR_MochiOrb.mat", orbShader);
                // linear colour space: keep these well below 1 or the ball clips to a flat white disc
                orb.SetColor("_Color", new Color(0.85f, 0.8f, 0.74f));
                orb.SetColor("_Shade", new Color(0.32f, 0.28f, 0.3f));
                orb.SetColor("_RimColor", new Color(1f, 0.9f, 0.72f));
                orb.SetFloat("_Glow", 0.06f);
                EditorUtility.SetDirty(orb);
                story.orbMaterial = orb;
            }
            else Debug.LogError("[Moon Rabbit] MoonRabbit/MochiOrb shader missing (Assets/_Project/World/Shaders).");
            story.quadMesh = Resources.GetBuiltinResource<Mesh>("Quad.fbx");
            story.sphereMesh = Resources.GetBuiltinResource<Mesh>("Sphere.fbx");

            var moon = root.Find("MR_Moon");
            if (moon != null)
            {
                story.moon = moon.GetComponent<Renderer>();
                var mg = moon.Find("Glow");
                if (mg != null) story.moonGlow = mg.GetComponent<Renderer>();
            }
            var earthT = root.Find("MR_Earth");
            if (earthT != null)
            {
                story.earth = earthT.GetComponent<Renderer>();
                var eg = earthT.Find("Glow");
                if (eg != null) story.earthGlow = eg.GetComponent<Renderer>();
            }
            story.moonFollowsProgress = s.moonFollowsProgress;
            var island = new List<Renderer>();
            foreach (var n in new[] { "MR_IslandUnderside", "MR_FloatingRocks" })
            {
                var t = root.Find(n);
                if (t != null) island.Add(t.GetComponent<Renderer>());
            }
            story.islandRenderers = island.ToArray();
            story.moonlight = moonlight;
            story.atmosphere = atm;

            var lanterns = new List<Renderer>(); var lights = new List<Light>(); var pools = new List<Renderer>();
            for (int i = 0; ; i++)
            {
                var l = root.Find($"MR_StoneLantern_{i}");
                if (l == null) break;
                lanterns.Add(l.GetComponent<Renderer>());
                lights.Add(l.GetComponentInChildren<Light>());
                var pool = root.Find($"MR_LanternPool_{i}");
                pools.Add(pool != null ? pool.GetComponent<Renderer>() : null);
            }
            story.lanterns = lanterns.ToArray();
            story.lanternLights = lights.ToArray();
            story.lanternPools = pools.ToArray();

            var tree = root.Find("MR_CassiaTree");
            if (tree != null) story.cassiaTree = tree.GetComponent<Renderer>();
            story.blossomGlowColor = Color.Lerp(s.cassiaGold, s.lantern, 0.4f);
            story.blossomPoints = blossomPoints.ToArray();
            story.stonePoints = stonePoints.ToArray();
            story.stoneGlowColor = s.moonlight;
            var grove = root.GetComponentInChildren<BambooGrove>();
            story.bamboo = grove;

            // follow the team's MochiStation if it is moved at runtime (UsuTablePlacer: real table from the room scan)
            var follow = go.AddComponent<StationFollower>();
            var ped = root.Find("MR_UsuPedestal");
            if (ped != null) follow.pedestal = ped;
            follow.pedestalHeight = s.pedestalHeight;
            var wl = root.Find("MR_MochiWarmLight");
            if (wl != null)
            {
                follow.warmLight = wl;
                follow.warmLightOffset = wl.localPosition - (s.mortarPosition + Vector3.up * s.pedestalHeight);
            }

            var snd = go.AddComponent<StorySounds>();
            snd.soundsEnabled = s.storySounds;
            snd.textSounds = s.textSounds;
            snd.progressSounds = s.progressSounds;
            snd.strikeSound = s.strikeSound;
            snd.textVolume = s.textSoundVolume;
            snd.progressSfxVolume = s.progressSfxVolume;
            snd.introLineClip = s.introLineClip; snd.instructionClip = s.instructionClip; snd.endingClip = s.endingClip;
            snd.strikeClip = s.strikeClip; snd.milestoneClip = s.milestoneClip; snd.fullMoonClip = s.fullMoonClip;
            snd.flightClip = s.flightClip; snd.arrivalClip = s.arrivalClip; snd.wishClip = s.wishClip;

            go.AddComponent<HeightDiagnostics>();
            var hr = go.AddComponent<HeadsetRestart>();
            hr.restartOnRemount = s.headsetRestart;
            hr.minRemovedSeconds = s.headsetRestartMinSeconds;

            var intro = go.AddComponent<StoryIntro>();
            intro.playIntro = s.storyIntro;
#if MR_TMP
            var fonts = StoryFontBuilder.Build();
            intro.narration.font = fonts.intro; intro.narration.material = fonts.introMat;
            intro.instruction.font = fonts.instruction; intro.instruction.material = fonts.instructionMat;
            story.endingStyle.font = fonts.ending; story.endingStyle.material = fonts.endingMat;
#endif
            var fadeShader = Shader.Find("MoonRabbit/ScreenFade");
            if (fadeShader != null)
            {
                var fm = MRAssets.GetMaterial($"{G}/Materials/MR_ScreenFade.mat", fadeShader);
                fm.SetColor("_Color", new Color(0f, 0f, 0f, 1f));
                EditorUtility.SetDirty(fm);
                intro.fadeMaterial = fm;
            }
            else Debug.LogError("[Moon Rabbit] MoonRabbit/ScreenFade shader missing (Assets/_Project/World/Shaders).");
        }

        // ------------------------------------------------------------------ warm light on the dough

        static void BuildMochiLight(MoonRabbitEnvironmentSettings s, Transform root)
        {
            bool teamUsu = s.pedestal && MoonRabbitMenu.PropBuilder == null;
            var dough = s.mortarPosition + Vector3.up * (teamUsu ? s.pedestalHeight + s.usuBase + 0.02f
                                                                 : s.mortarHeight - s.hollowDepth + 0.02f);
            var go = new GameObject("MR_MochiWarmLight");
            go.transform.SetParent(root, false);
            go.transform.localPosition = dough + s.mochiLightOffset;
            var aim = dough - go.transform.localPosition;
            if (aim.sqrMagnitude > 1e-6f) go.transform.localRotation = Quaternion.LookRotation(aim);
            var l = go.AddComponent<Light>();
            l.type = s.mochiLightSpotAngle > 0f ? LightType.Spot : LightType.Point;
            if (l.type == LightType.Spot)
            {
                l.spotAngle = s.mochiLightSpotAngle;
                l.innerSpotAngle = s.mochiLightSpotAngle * 0.4f;
            }
            l.color = s.mochiLightColor;
            l.intensity = s.mochiLightIntensity;
            l.range = s.mochiLightRange;
            l.shadows = LightShadows.None;
            l.renderMode = LightRenderMode.ForcePixel; // Important: otherwise a vertex light, see the setting's tooltip
            if (s.warmLightExcludedLayer >= 0) l.cullingMask = ~(1 << s.warmLightExcludedLayer);
            l.lightmapBakeType = LightmapBakeType.Realtime;
        }

        // ------------------------------------------------------------------ stepping stones

        static void BuildStones(MoonRabbitEnvironmentSettings s, Transform root, Mats m, Rng rng)
        {
            stonePoints.Clear();
            if (s.stoneCount <= 0) return;
            // island: the path runs on to the edge and stops at the drop, pointing out into the sky
            float pathEnd = s.floatingIsland ? IslandEdge(s, MoonRabbitEnvironmentSettings.Azimuth(s.stonePathDirection).x, MoonRabbitEnvironmentSettings.Azimuth(s.stonePathDirection).z) - 1.1f : s.stonePathEnd;
            int stoneCount = s.floatingIsland ? Mathf.Max(s.stoneCount, Mathf.RoundToInt((pathEnd - s.stonePathStart) / 0.95f)) : s.stoneCount;
            var mb = new MeshBuilder(1);
            for (int i = 0; i < stoneCount; i++)
            {
                float t = stoneCount == 1 ? 0f : (float)i / (stoneCount - 1);
                float r = Mathf.Lerp(s.stonePathStart, pathEnd, t);
                float az = s.stonePathDirection + Mathf.Sin(t * Mathf.PI * 2f) * 10f;
                var p = MoonRabbitEnvironmentSettings.Azimuth(az) * r + new Vector3(rng.Range(-0.1f, 0.1f), 0f, rng.Range(-0.1f, 0.1f));
                float size = rng.Range(0.3f, 0.45f) * Mathf.Lerp(1f, 0.8f, t);
                p.y = GroundHeight(s, p.x, p.z);
                mb.Blob(p, new Vector3(size, size * 0.22f, size * rng.Range(0.75f, 1f)), 1, 0.15f, rng.Raw, 0);
                stonePoints.Add(p + Vector3.up * size * 0.22f);
            }
            var mesh = MRAssets.SaveMesh(mb.ToMesh("MR_SteppingStones"), $"{G}/Meshes/MR_SteppingStones.asset");
            var go = MeshObject("MR_SteppingStones", root, mesh, new[] { m.stone }, ShadowCastingMode.Off);
            GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic);
        }

        // ------------------------------------------------------------------ usu pedestal

        /// <summary>Tree stump on a stone slab at the mortar position. Its top (pedestalHeight) is where the
        /// team's usu (MochiStation) stands, since full VR has no real table.</summary>
        static void BuildPedestal(MoonRabbitEnvironmentSettings s, Transform root, Mats m, Rng rng)
        {
            var mb = new MeshBuilder(3); // 0 bark, 1 cut top, 2 stone
            var up = Vector3.up;
            float H = Mathf.Max(0.2f, s.pedestalHeight);
            float R = Mathf.Max(0.15f, s.pedestalTopRadius);
            const float slab = 0.07f;

            // stone slab, slightly sunk into the ground, plus a few loose stones
            mb.Frustum(up * -0.02f, up * slab, R + 0.26f, R + 0.2f, 7, 2, false, true, rng.Range(0f, 1f));
            for (int i = 0; i < 3; i++)
            {
                var d = MoonRabbitEnvironmentSettings.Azimuth(60f + i * 110f + rng.Range(-20f, 20f));
                float size = rng.Range(0.07f, 0.11f);
                mb.Blob(d * (R + rng.Range(0.33f, 0.42f)) + up * 0.01f, new Vector3(size, size * 0.5f, size), 1, 0.15f, rng.Raw, 2);
            }

            // stump: slightly wider at the bottom, cut top in light wood
            mb.Frustum(up * (slab - 0.01f), up * (H - 0.015f), R * 1.08f, R, 11, 0, false, false);
            mb.Frustum(up * (H - 0.015f), up * H, R, R * 0.97f, 11, 1, false, true);
            for (int i = 0; i < 4; i++) // root flare over the slab
            {
                var d = MoonRabbitEnvironmentSettings.Azimuth(i * 90f + 30f + rng.Range(-15f, 15f));
                mb.Frustum(d * R * 0.85f + up * Mathf.Min(0.28f, H * 0.4f), d * (R + 0.17f) + up * (slab + 0.005f), 0.08f, 0.03f, 5, 0, false, true);
            }

            var mesh = MRAssets.SaveMesh(mb.ToMesh("MR_UsuPedestal"), $"{G}/Meshes/MR_UsuPedestal.asset");
            var go = MeshObject("MR_UsuPedestal", root, mesh, new[] { m.bark, m.woodCut, m.stone }, ShadowCastingMode.On);
            go.transform.localPosition = new Vector3(s.mortarPosition.x, 0f, s.mortarPosition.z);
            var col = go.AddComponent<BoxCollider>();
            col.center = new Vector3(0f, H * 0.5f, 0f);
            col.size = new Vector3(R * 2f, H, R * 2f);
            // not static: StationFollower moves/stretches it when the team's MochiStation is placed on a real table
            GameObjectUtility.SetStaticEditorFlags(go, 0);
        }

        // ------------------------------------------------------------------ moonlight

        static Light BuildMoonlight(MoonRabbitEnvironmentSettings s, Transform root)
        {
            var go = new GameObject("MR_Moonlight");
            go.transform.SetParent(root, false);
            // Light comes from the Moon's side (behind the tree), so the scene is rim-lit like an ink painting.
            var from = MoonRabbitEnvironmentSettings.SkyDirection(s.moonAzimuth, s.moonlightPitch);
            go.transform.rotation = Quaternion.LookRotation(-from);
            var l = go.AddComponent<Light>();
            l.type = LightType.Directional;
            l.color = s.moonlightColor;
            l.intensity = s.moonlightIntensity;
            l.shadows = LightShadows.Soft;
            l.shadowStrength = 0.7f;
            return l;
        }

        // ------------------------------------------------------------------ sky

        static Material BuildSky(MoonRabbitEnvironmentSettings s, Rng rng)
        {
            int W = Mathf.ClosestPowerOfTwo(Mathf.Clamp(s.skyTextureWidth, 512, 4096));
            int H = W / 2;
            var px = new Color[W * H];

            for (int y = 0; y < H; y++)
            {
                float elev = ((y + 0.5f) / H - 0.5f) * 180f;
                Color c;
                if (s.floatingIsland)
                    c = Color.Lerp(s.navySky, s.skyZenith, Mathf.Pow(Mathf.Abs(elev) / 90f, 0.7f) * (elev < 0f ? 0.85f : 1f)); // open sky all round
                else
                {
                    c = elev <= 0f ? s.horizonFog : Color.Lerp(s.navySky, s.skyZenith, Mathf.Pow(elev / 90f, 0.7f));
                    if (elev > 0f && elev < 4f) c = Color.Lerp(s.horizonFog, c, elev / 4f); // dark forest-floor haze meets the sky
                }
                float mist = Mathf.Exp(-Mathf.Pow((elev - 5f) / 6f, 2f)) * 0.07f; // pale ink-wash band above the horizon
                c = Color.Lerp(c, s.moonlight, mist);
                for (int x = 0; x < W; x++) px[y * W + x] = c;
            }

            float minSin = Mathf.Sin((s.floatingIsland ? -84f : 6f) * Mathf.Deg2Rad);
            int stars = s.floatingIsland ? Mathf.RoundToInt(s.starCount * 1.8f) : s.starCount; // the lower half of the sky too
            for (int i = 0; i < stars; i++)
            {
                float elev = Mathf.Asin(rng.Range(minSin, 1f));
                float cx = rng.Value * W;
                float cy = (elev / Mathf.PI + 0.5f) * H;
                float b = Mathf.Pow(rng.Value, 3f) * 0.9f + 0.1f;
                if (!s.floatingIsland) b *= Mathf.SmoothStep(0f, 1f, Mathf.InverseLerp(6f, 22f, elev * Mathf.Rad2Deg));
                else b *= Mathf.Lerp(0.45f, 1f, Mathf.InverseLerp(0f, 20f, Mathf.Abs(elev * Mathf.Rad2Deg))); // softer near the horizon haze
                float size = 0.55f + b * 1.3f;
                float sx = size / Mathf.Max(0.12f, Mathf.Cos(elev));
                var tint = Color.Lerp(Color.white, rng.Value < 0.2f ? s.lantern : s.moonlight, 0.35f) * b;
                int rx = Mathf.CeilToInt(sx * 2.5f), ry = Mathf.CeilToInt(size * 2.5f);
                for (int dy = -ry; dy <= ry; dy++)
                {
                    int yy = Mathf.FloorToInt(cy) + dy;
                    if (yy < 0 || yy >= H) continue;
                    for (int dx = -rx; dx <= rx; dx++)
                    {
                        int xx = ((Mathf.FloorToInt(cx) + dx) % W + W) % W;
                        float g = Mathf.Exp(-(dx * dx / (sx * sx) + dy * dy / (size * size)));
                        if (g < 0.01f) continue;
                        var o = px[yy * W + xx];
                        px[yy * W + xx] = new Color(Mathf.Min(1f, o.r + tint.r * g), Mathf.Min(1f, o.g + tint.g * g), Mathf.Min(1f, o.b + tint.b * g), 1f);
                    }
                }
            }

            var tex = new Texture2D(W, H, TextureFormat.RGBA32, false);
            tex.SetPixels(px);
            tex.Apply();
            var saved = MRAssets.SavePng(tex, $"{G}/Textures/MR_SkyPanorama.png", false, TextureWrapMode.Repeat, TextureWrapMode.Clamp, W);

            var shader = Shader.Find("Skybox/Panoramic");
            var mat = MRAssets.GetMaterial($"{G}/Materials/MR_Sky.mat", shader);
            mat.SetTexture("_MainTex", saved);
            mat.SetFloat("_Mapping", 1f);
            mat.DisableKeyword("_MAPPING_6_FRAMES_LAYOUT");
            mat.EnableKeyword("_MAPPING_LATITUDE_LONGITUDE_LAYOUT");
            mat.SetFloat("_ImageType", 0f);
            mat.SetFloat("_Exposure", 1f);
            EditorUtility.SetDirty(mat);
            return mat;
        }

        // ------------------------------------------------------------------ Moon and Earth

        static void BuildCelestial(MoonRabbitEnvironmentSettings s, Transform root)
        {
            var celestial = Shader.Find("MoonRabbit/Celestial");
            var glow = Shader.Find("MoonRabbit/Glow");
            if (celestial == null || glow == null)
            {
                Debug.LogError("[Moon Rabbit] MoonRabbit/Celestial or MoonRabbit/Glow shader missing (Assets/_Project/World/Shaders).");
                return;
            }
            var halo = HaloTexture();
            var quad = Resources.GetBuiltinResource<Mesh>("Quad.fbx");
            var crng = new Rng(s.seed + 17);

            // ---- Moon
            var craters = new List<(Vector3 dir, float size)>();
            for (int i = 0; i < 18; i++) craters.Add((RandomDir(crng, -1f, 1f), crng.Range(0.06f, 0.2f)));
            var moonBase = Color.Lerp(s.mochi, s.moonlight, 0.5f);
            var maria = Mul(s.moonlight, 0.74f);
            var moonMesh = Icosphere.SmoothSphere("MR_Moon", 3, p =>
            {
                float n = Noise3(p, 1.6f, 11f);
                var c = Color.Lerp(moonBase, maria, Mathf.SmoothStep(0f, 1f, Mathf.InverseLerp(0.52f, 0.62f, n)));
                foreach (var (dir, size) in craters)
                {
                    float d = Vector3.Angle(p, dir) * Mathf.Deg2Rad;
                    if (d < size) c = Color.Lerp(c, maria, 0.35f * (1f - d / size));
                }
                return c;
            });
            moonMesh = MRAssets.SaveMesh(moonMesh, $"{G}/Meshes/MR_Moon.asset");

            var moonDir = MoonRabbitEnvironmentSettings.SkyDirection(s.moonAzimuth, s.moonElevation);
            var moonMat = MRAssets.GetMaterial($"{G}/Materials/MR_Moon.mat", celestial);
            moonMat.SetColor("_Color", Color.white);
            moonMat.SetVector("_LightDir", (-moonDir + Vector3.up * 0.1f).normalized); // full Moon: lit side faces the player
            moonMat.SetFloat("_Ambient", 0.12f);
            moonMat.SetFloat("_Emission", 0.1f);
            moonMat.SetColor("_RimColor", s.moonlight);
            EditorUtility.SetDirty(moonMat);

            var moon = MeshObject("MR_Moon", root, moonMesh, new[] { moonMat }, ShadowCastingMode.Off, false);
            moon.transform.localPosition = moonDir * s.moonDistance;
            moon.transform.localRotation = Quaternion.LookRotation(-moonDir);
            moon.transform.localScale = Vector3.one * s.moonRadius;

            var moonGlowMat = MRAssets.GetMaterial($"{G}/Materials/MR_MoonGlow.mat", glow);
            moonGlowMat.SetTexture("_MainTex", halo);
            moonGlowMat.SetColor("_Color", s.moonlight);
            moonGlowMat.SetFloat("_Intensity", 0.8f);
            EditorUtility.SetDirty(moonGlowMat);
            var moonGlow = MeshObject("Glow", moon.transform, quad, new[] { moonGlowMat }, ShadowCastingMode.Off, false);
            moonGlow.transform.localPosition = Vector3.back * 1.1f; // just behind the Moon (local +Z faces the player)
            moonGlow.transform.localScale = Vector3.one * 4.2f;

            // ---- Earth
            var sand = new Color(0.79f, 0.70f, 0.49f);
            var earthMesh = Icosphere.SmoothSphere("MR_Earth", 3, p =>
            {
                float n = Noise3(p, 1.8f, 3f);
                Color c = Color.Lerp(s.earthOcean, Mul(s.earthOcean, 0.75f), Mathf.InverseLerp(0.5f, 0.3f, n));
                if (n > 0.53f) c = n < 0.555f ? sand : s.earthLand;
                if (Mathf.Abs(p.y) > 0.84f) c = Color.white;
                float cloud = Noise3(p, 3.3f, 7f);
                if (cloud > 0.6f) c = Color.Lerp(c, Color.white, Mathf.InverseLerp(0.6f, 0.72f, cloud) * 0.75f);
                return c;
            });
            earthMesh = MRAssets.SaveMesh(earthMesh, $"{G}/Meshes/MR_Earth.asset");

            var earthDir = MoonRabbitEnvironmentSettings.SkyDirection(s.earthAzimuth, s.earthElevation);
            var earthMat = MRAssets.GetMaterial($"{G}/Materials/MR_Earth.mat", celestial);
            earthMat.SetColor("_Color", Color.white);
            earthMat.SetVector("_LightDir", (-earthDir * 0.6f + MoonRabbitEnvironmentSettings.Azimuth(s.earthAzimuth + 90f) * 0.8f).normalized);
            earthMat.SetFloat("_Ambient", 0.18f);
            earthMat.SetFloat("_Emission", 0.45f);
            earthMat.SetColor("_RimColor", new Color(0.55f, 0.75f, 1f));
            EditorUtility.SetDirty(earthMat);

            var earth = MeshObject("MR_Earth", root, earthMesh, new[] { earthMat }, ShadowCastingMode.Off, false);
            earth.transform.localPosition = earthDir * s.earthDistance;
            earth.transform.localRotation = Quaternion.LookRotation(-earthDir) * Quaternion.Euler(-20f, 0f, 23f);
            earth.transform.localScale = Vector3.one * s.earthRadius;

            var earthGlowMat = MRAssets.GetMaterial($"{G}/Materials/MR_EarthGlow.mat", glow);
            earthGlowMat.SetTexture("_MainTex", halo);
            earthGlowMat.SetColor("_Color", new Color(0.45f, 0.65f, 1f));
            earthGlowMat.SetFloat("_Intensity", 0.5f);
            EditorUtility.SetDirty(earthGlowMat);
            var earthGlow = MeshObject("Glow", earth.transform, quad, new[] { earthGlowMat }, ShadowCastingMode.Off, false);
            // undo the Earth's tilt so the halo faces the player
            earthGlow.transform.localRotation = Quaternion.Inverse(Quaternion.Euler(-20f, 0f, 23f));
            earthGlow.transform.localPosition = earthGlow.transform.localRotation * (Vector3.back * 1.1f);
            earthGlow.transform.localScale = Vector3.one * 3.2f;
        }

        static Texture2D HaloTexture()
        {
            const int N = 128;
            var tex = new Texture2D(N, N, TextureFormat.RGBA32, false);
            var px = new Color[N * N];
            for (int y = 0; y < N; y++)
                for (int x = 0; x < N; x++)
                {
                    float dx = (x + 0.5f) / N - 0.5f, dy = (y + 0.5f) / N - 0.5f;
                    float d = Mathf.Sqrt(dx * dx + dy * dy) / 0.5f;
                    float a = Mathf.Pow(Mathf.Clamp01(1f - d), 2.4f);
                    px[y * N + x] = new Color(a, a, a, 1f);
                }
            tex.SetPixels(px);
            tex.Apply();
            return MRAssets.SavePng(tex, $"{G}/Textures/MR_Halo.png", true, TextureWrapMode.Clamp, TextureWrapMode.Clamp, N);
        }

        // ------------------------------------------------------------------ helpers

        public static GameObject MeshObject(string name, Transform parent, Mesh mesh, Material[] mats,
                                            ShadowCastingMode shadows, bool receiveShadows = true)
        {
            var go = new GameObject(name);
            if (parent != null) go.transform.SetParent(parent, false);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            var mr = go.AddComponent<MeshRenderer>();
            mr.sharedMaterials = mats;
            mr.shadowCastingMode = shadows;
            mr.receiveShadows = receiveShadows;
            mr.lightProbeUsage = LightProbeUsage.Off;
            mr.reflectionProbeUsage = ReflectionProbeUsage.Off;
            return go;
        }

        public static Color Mul(Color c, float f) => new Color(c.r * f, c.g * f, c.b * f, 1f);

        static Vector3 RandomDir(Rng rng, float minY, float maxY)
        {
            for (int i = 0; i < 64; i++)
            {
                var v = new Vector3(rng.Range(-1f, 1f), rng.Range(-1f, 1f), rng.Range(-1f, 1f));
                float m = v.magnitude;
                if (m < 0.05f || m > 1f) continue;
                v /= m;
                if (v.y >= minY && v.y <= maxY) return v;
            }
            return Vector3.up;
        }

        /// <summary>Stalks actually placed by the last build (after the back thinning).</summary>
        public static int BambooPlaced;

        static float Hash01(Vector3 p) { float v = Mathf.Sin(p.x * 12.9898f + p.z * 78.233f) * 43758.5453f; return v - Mathf.Floor(v); }

        static float Noise3(Vector3 p, float freq, float seed)
        {
            p = p * freq + Vector3.one * (seed * 7.31f + 50f);
            float n = (Mathf.PerlinNoise(p.x, p.y) + Mathf.PerlinNoise(p.y, p.z) + Mathf.PerlinNoise(p.z, p.x)) / 3f;
            return Mathf.Clamp01(0.5f + (n - 0.5f) * 1.8f);
        }

        public class Rng
        {
            public readonly System.Random Raw;
            public Rng(int seed) { Raw = new System.Random(seed); }
            public float Value => (float)Raw.NextDouble();
            public float Range(float a, float b) => a + (b - a) * Value;
            public float Range(Vector2 v) => Range(v.x, v.y);
            public int Int(int minInclusive, int maxExclusive) => Raw.Next(minInclusive, maxExclusive);
        }
    }
}
