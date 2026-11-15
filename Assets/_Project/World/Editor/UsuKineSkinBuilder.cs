using System.Collections.Generic;
using UnityEngine;

namespace MoonRabbit.WorldEditor
{
    /// <summary>
    /// Meshes and materials that re-skin the team's usu and kine (MochiPound: UsuTool / KineTool) in the style of
    /// Jihyeon's mortar and pestle. Only assets are generated here; they are attached in the scene as children of
    /// the UsuTool and KineTool transforms, so they move with them. Sizes come from the settings, which must
    /// match the colliders:
    ///  - usu: origin = bottom resting on the pedestal, +Y up. The dough floor is at usuBase and the dough meets an
    ///    analytic cone wall from usuInnerBottom (floor) to usuInnerTop (floor + usuDepth), so the inner wall here
    ///    is exactly that cone and the rim is at usuBase + usuDepth.
    ///  - kine: origin = centre of the flat striking face, -Y = strike direction, head along +Y. KineDriver holds
    ///    the handle end kineHandleLength behind the head centre along local -Z (controller.forward).
    /// </summary>
    public static class UsuKineSkinBuilder
    {
        const string G = MRAssets.WorldGenerated;
        public const string UsuMeshPath = G + "/Meshes/MR_UsuSkin.asset";
        public const string KineMeshPath = G + "/Meshes/MR_KineSkin.asset";

        public static void Build(MoonRabbitEnvironmentSettings s)
        {
            MRAssets.Lit(G, "MR_UsuWood", s.wood, 0.15f);
            MRAssets.Lit(G, "MR_UsuEndGrain", s.lightWood, 0.12f);
            MRAssets.Lit(G, "MR_UsuGrainRing", Color.Lerp(s.wood, s.trunk, 0.35f), 0.1f);
            MRAssets.Lit(G, "MR_SkinRabbitPink", s.rabbitPink, 0.2f);
            MRAssets.Lit(G, "MR_KineHandle", s.lightWood, 0.2f);
            MRAssets.Lit(G, "MR_KineHead", Color.Lerp(s.lightWood, s.mochi, 0.35f), 0.25f);

            MRAssets.SaveMesh(BuildUsu(s), UsuMeshPath);
            MRAssets.SaveMesh(BuildKine(s), KineMeshPath);
        }

        /// <summary>Material order for the usu skin's sub-meshes.</summary>
        public static string[] UsuMaterials => new[] { "MR_UsuWood", "MR_UsuEndGrain", "MR_UsuGrainRing", "MR_SkinRabbitPink" };
        /// <summary>Material order for the kine skin's sub-meshes.</summary>
        public static string[] KineMaterials => new[] { "MR_KineHandle", "MR_KineHead", "MR_SkinRabbitPink" };

        // Flat facets: put the vertices on r / cos(pi/n) so the middle of every facet lies on the true radius.
        static float Facet(float r, int sides) => r / Mathf.Cos(Mathf.PI / sides);

        static Mesh BuildUsu(MoonRabbitEnvironmentSettings s)
        {
            const int N = 32;
            float R = s.usuOuterRadius, b = s.usuBase, top = s.usuBase + s.usuDepth;
            // The visible bowl sits a little outside the solver's cone (and below its floor): the dough's surface hugs the
            // cone, and a wall exactly on it z-fights with the dough (two materials flickering at the rim on Quest).
            const float WallGap = 0.008f, FloorGap = 0.006f;
            float rt = Facet(s.usuInnerTop + WallGap, N), rb = Facet(s.usuInnerBottom + WallGap, N);
            float Ro = Facet(R, N);
            var mb = new MeshBuilder(4); // 0 wood, 1 end grain (rim top), 2 grain ring, 3 pink band

            // Log mortar like the kit's: foot, slight waist, flaring up to a rounded rim (outer wall walked upward).
            var outer = new List<Vector2>
            {
                new Vector2(0f, 0f),
                new Vector2(Ro * 0.90f, 0f),
                new Vector2(Ro * 0.95f, 0.008f),
                new Vector2(Ro * 0.93f, top * 0.22f),
                new Vector2(Ro * 0.87f, top * 0.48f),
                new Vector2(Ro * 0.93f, top * 0.78f),
                new Vector2(Ro, top - 0.012f),
                new Vector2(Ro * 0.985f, top),
            };
            mb.Lathe(outer, N, 0);

            // Rim top: end grain with one darker growth ring.
            float rimIn = rt + 0.006f;
            float ringA = Mathf.Lerp(rimIn, Ro * 0.985f, 0.45f), ringB = ringA + 0.008f;
            mb.Lathe(new[] { new Vector2(Ro * 0.985f, top), new Vector2(ringB, top) }, N, 1);
            mb.Lathe(new[] { new Vector2(ringB, top), new Vector2(ringA, top) }, N, 2);
            mb.Lathe(new[] { new Vector2(ringA, top), new Vector2(rimIn, top) }, N, 1);

            // Inner wall = the solver's cone pushed out by WallGap (rim inner chamfer just outside it), then the floor.
            mb.Lathe(new[]
            {
                new Vector2(rimIn, top), new Vector2(rt, top - 0.004f),
                new Vector2(Mathf.Lerp(rb, rt, 0.5f), b + s.usuDepth * 0.5f),
                new Vector2(rb, b - FloorGap), new Vector2(0f, b - FloorGap),
            }, N, 0);

            // Pink rope band around the waist (as on the kit mortar).
            float bandY = top * 0.48f, bandR = Ro * 0.87f + 0.006f;
            mb.Frustum(Vector3.up * (bandY - 0.011f), Vector3.up * (bandY + 0.011f), bandR, bandR, N, 3, false, false); // no caps: they would cross the bowl
            return mb.ToMesh("MR_UsuSkin");
        }

        static Mesh BuildKine(MoonRabbitEnvironmentSettings s)
        {
            const int N = 16;
            float r = Facet(s.kineHeadRadius, N), L = s.kineHeadLength, fr = s.kineFaceRounding;
            var up = Vector3.up;
            var mb = new MeshBuilder(3); // 0 handle, 1 head, 2 pink

            // Head: flat face at y = 0 (radius r - rounding), rounded edge, straight body, chamfered top.
            mb.Frustum(Vector3.zero, up * fr * 0.35f, r - fr, r - fr * 0.3f, N, 1, true, false);
            mb.Frustum(up * fr * 0.35f, up * fr, r - fr * 0.3f, r, N, 1, false, false);
            mb.Frustum(up * fr, up * (L - fr), r, r, N, 1, false, false);
            mb.Frustum(up * (L - fr), up * L, r, r - fr, N, 1, false, true);
            // Two pink bands near the ends of the head (like the kit pestle's wraps).
            foreach (float y in new[] { fr + 0.012f, L - fr - 0.012f })
                mb.Frustum(up * (y - 0.007f), up * (y + 0.007f), r + 0.003f, r + 0.003f, N, 2, false, false);

            // Handle: from inside the head's middle back along -Z to the controller (handle end), then a knob.
            float hw = 0.016f, mid = L * 0.5f, H = s.kineHandleLength;
            var a = new Vector3(0f, mid, -r * 0.5f);
            var e = new Vector3(0f, mid, -H);
            mb.Frustum(a, e, hw * 1.15f, hw, 8, 0, false, false);
            mb.Frustum(e, e + Vector3.back * 0.025f, hw * 1.35f, hw * 0.7f, 8, 0, true, true);
            // Pink grip wrap where the hand holds (the last 12 cm before the end).
            mb.Frustum(new Vector3(0f, mid, -H + 0.13f), new Vector3(0f, mid, -H + 0.01f), hw * 1.25f, hw * 1.2f, 8, 2, true, true);
            return mb.ToMesh("MR_KineSkin");
        }
    }
}
