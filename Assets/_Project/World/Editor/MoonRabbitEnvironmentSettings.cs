using UnityEngine;

namespace MoonRabbit.WorldEditor
{
    /// <summary>
    /// Every count, radius and color the builder uses. Edit the asset
    /// (Moon Rabbit > Select Build Settings), then run Moon Rabbit > Build Environment again.
    /// Layout: the mortar is at the origin, the player faces +Z, the Moon and the
    /// bamboo opening are toward +Z (azimuth 0), angles go clockwise seen from above.
    /// </summary>
    public class MoonRabbitEnvironmentSettings : ScriptableObject
    {
        [Header("General")]
        public int seed = 2026;

        [Header("Palette")]
        public Color navySky = Hex(0x1E2A4A);
        public Color moonlight = Hex(0xDDE6F0);
        public Color bambooLight = Hex(0x4F7A55);
        public Color bambooDark = Hex(0x2F4A3A);
        public Color cassiaGold = Hex(0xE8C46A);
        public Color lantern = Hex(0xF2A65A);
        public Color mochi = Hex(0xFAF7F0);
        public Color rabbitPink = Hex(0xF2B8C6);
        [Tooltip("Ground outside the clearing.")]
        public Color darkGround = Hex(0x253047);
        public Color stone = Hex(0x8E97A6);
        public Color wood = Hex(0xA07E5E);
        public Color lightWood = Hex(0xC9A77C);
        public Color trunk = Hex(0x3E3230);
        public Color earthOcean = Hex(0x3C6DB4);
        public Color earthLand = Hex(0x6E9F5A);

        [Header("Clearing")]
        [Tooltip("Radius of the silver clearing (m).")]
        public float clearingRadius = 4f;
        public int clearingSides = 28;
        [Tooltip("Dark ground disc radius; its edge disappears in the fog.")]
        public float groundRadius = 40f;

        [Header("Bamboo ring")]
        public int bambooCount = 160;
        [Range(1, 8)] public int bambooVariants = 4;
        public float bambooInnerRadius = 5f;
        public float bambooOuterRadius = 15f;
        public Vector2 bambooHeight = new Vector2(5.5f, 9.5f);
        public Vector2 bambooStalkRadius = new Vector2(0.045f, 0.075f);
        public float bambooSegmentLength = 0.75f;
        public int leavesPerStalk = 12;
        [Tooltip("Azimuth (deg) of the gap in the ring that opens the view to the Moon and Earth.")]
        public float openingDirection = 0f;
        [Tooltip("Width (deg) of that gap.")]
        public float openingWidth = 55f;
        [Tooltip("Stalks lean outward (deg) so the sky above the clearing stays open.")]
        public float outwardLean = 5f;
        [Tooltip("On: one BambooGrove draws everything with Graphics.RenderMeshInstanced. Off: one MeshRenderer per stalk.")]
        public bool useGpuInstancing = true;

        [Header("Cassia tree")]
        public Vector3 treePosition = new Vector3(0.4f, 0f, 3.2f);
        public float treeHeight = 3.6f;
        public float canopyRadius = 1.6f;
        public int canopyBlobs = 7;
        public int blossomCount = 28;

        [Header("Stone lanterns")]
        public Vector3[] lanternPositions = { new Vector3(-2.4f, 0f, 1.5f), new Vector3(2.6f, 0f, 1.2f) };
        public float lanternLightRange = 4f;
        public float lanternLightIntensity = 1.0f;

        [Header("Stepping stones")]
        public int stoneCount = 8;
        [Tooltip("Azimuth (deg) of the stepping-stone path.")]
        public float stonePathDirection = -12f;
        public float stonePathStart = 4.4f;
        public float stonePathEnd = 11f;

        [Header("Moon and Earth (fog-free, far away)")]
        public float moonAzimuth = 0f;
        public float moonElevation = 24f;
        public float moonDistance = 140f;
        public float moonRadius = 16f;
        public float earthAzimuth = -38f;
        public float earthElevation = 40f;
        public float earthDistance = 130f;
        public float earthRadius = 4.5f;

        [Header("Sky")]
        public int starCount = 1400;
        [Tooltip("Width of the generated panoramic sky texture (height is half).")]
        public int skyTextureWidth = 2048;
        public Color skyZenith = Hex(0x111930);

        [Header("Moonlight, ambient and fog")]
        public Color moonlightColor = Hex(0xA9C1E8);
        public float moonlightIntensity = 0.9f;
        [Tooltip("Height of the moonlight above the horizon (deg). It comes from the Moon's side, behind the tree.")]
        public float moonlightPitch = 35f;
        public FogMode fogMode = FogMode.ExponentialSquared;
        public float fogDensity = 0.04f;
        public float fogStart = 6f;
        public float fogEnd = 40f;
        public Color ambientSky = new Color(0.36f, 0.42f, 0.60f);
        public Color ambientEquator = new Color(0.26f, 0.30f, 0.44f);
        public Color ambientGround = new Color(0.10f, 0.11f, 0.16f);

        [Header("Mortar")]
        public Vector3 mortarPosition = Vector3.zero;
        public float mortarOuterDiameter = 0.6f;
        public float mortarHeight = 0.65f;
        public float hollowDiameter = 0.3f;
        public float hollowDepth = 0.3f;
        [Tooltip("Show a mochi blob in the bowl until the real VR-Doh dough is in.")]
        public bool doughPlaceholder = true;

        [Header("Pestle")]
        public Vector3 pestleRestPosition = new Vector3(0.5f, 0f, -0.2f);
        public float handleLength = 1.0f;
        public float handleDiameter = 0.05f;
        public float headDiameter = 0.14f;
        public float headLength = 0.22f;
        public float pestleMass = 2.5f;

        [Header("Test rig (sandbox scene only)")]
        public Vector3 playerPosition = new Vector3(0f, 0f, -0.8f);

        public static Color Hex(int rgb) =>
            new Color(((rgb >> 16) & 0xFF) / 255f, ((rgb >> 8) & 0xFF) / 255f, (rgb & 0xFF) / 255f, 1f);

        /// <summary>Direction on the XZ plane for an azimuth in degrees (0 = +Z, 90 = +X).</summary>
        public static Vector3 Azimuth(float deg) =>
            new Vector3(Mathf.Sin(deg * Mathf.Deg2Rad), 0f, Mathf.Cos(deg * Mathf.Deg2Rad));

        public static Vector3 SkyDirection(float azimuth, float elevation)
        {
            float e = elevation * Mathf.Deg2Rad;
            return Azimuth(azimuth) * Mathf.Cos(e) + Vector3.up * Mathf.Sin(e);
        }
    }
}
