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
        [Tooltip("Forest floor outside the clearing: dark moss/earth, so it does not read as water.")]
        public Color darkGround = Hex(0x2E3026);
        [Tooltip("Fog and the sky below the horizon. Darker and greyer than navySky, so the far ground reads as land.")]
        public Color horizonFog = Hex(0x1C2230);
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
        public int bambooCount = 300;
        [Range(1, 8)] public int bambooVariants = 4;
        public float bambooInnerRadius = 4.8f;
        [Tooltip("Radial distribution of clumps: 1 = even in radius (denser near the clearing than area-uniform), higher = denser still.")]
        public float bambooInnerBias = 1.15f;
        [Tooltip("How deep the stalk bases sit in the ground / earth mound (m).")]
        public float bambooSink = 0.12f;
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

        [Tooltip("Share of bamboo left out behind the player (the half facing away from the Moon), fading in from the sides. " +
                 "The forest there is rarely looked at; fewer stalks help Quest's frame time.")]
        [Range(0f, 0.8f)] public float bambooBackThinning = 0.36f;

        [Header("Team props (tzhu235's models in Assets/, placed only when present)")]
        [Tooltip("Paper lanterns, wind chime, bench, sign, fountain, mossy stones and fireflies from the team's models " +
                 "(Assets/<name>.obj). Skipped when a model is missing (e.g. in the kit project).")]
        public bool teamProps = true;
        [Tooltip("Counts: the paper lantern stand is ~56k triangles each (rope detail), so one stand by default.")]
        [Range(0, 3)] public int teamLanternStands = 1;
        [Range(0, 4)] public int teamMossyStones = 1;
        [Range(0, 3)] public int teamFireflySwarms = 1;

        [Header("Clear view to the Earth (the ending's flight)")]
        [Tooltip("Bamboo in the sector toward the Earth (seen from the player) is lower, so the mochi's flight and the Earth stay visible.")]
        public bool earthViewClear = true;
        [Tooltip("Half width of the fully lowered sector (deg) and the extra width over which bamboo grows back to full height.")]
        public float earthViewHalfWidth = 24f;
        public float earthViewFalloff = 22f;
        [Tooltip("Stalk tops in the sector stay below this elevation seen from the eye (deg); the flight starts low and climbs.")]
        public float earthViewMaxElevation = 13f;
        [Tooltip("Eye height above the ground at the player position (m), for the sight line.")]
        public float earthViewEyeHeight = 1.9f;

        [Header("Cassia tree")]
        [Tooltip("Front-right of the player, so the canopy does not hide the Moon (azimuth 0) from the usu.")]
        public Vector3 treePosition = new Vector3(2.4f, 0f, 2.8f);
        public float treeHeight = 3.6f;
        public float canopyRadius = 1.6f;
        public int canopyBlobs = 7;
        public int blossomCount = 28;

        [Header("Stone lanterns")]
        public Vector3[] lanternPositions = { new Vector3(-2.4f, 0f, 1.5f), new Vector3(2.6f, 0f, 1.2f) };
        public float lanternLightRange = 4f;
        public float lanternLightIntensity = 1.0f;
        [Tooltip("Fake pool of lantern light on the ground under each lantern: additive MoonRabbit/Glow quad, no lighting cost.")]
        public bool lanternGlowPools = true;
        public float lanternPoolRadius = 1.3f;
        [Range(0f, 1f)] public float lanternPoolIntensity = 0.6f;

        [Header("Story progression (MR_Story in MR_Environment)")]
        [Tooltip("MoonProgress (0..1) + demo strike detection + rice-to-mochi grains + the night coming alive + the ending.")]
        public bool storyProgression = true;
        [Min(1)] public int demoStrikesToFull = 12;
        [Range(0, 3000)] public int riceGrainCount = 2400;
        [Tooltip("On: the Moon waxes and brightens with progress. Off: a dark crescent until the mochi reaches the Earth.")]
        public bool moonFollowsProgress = true;
        [Tooltip("Intro (fade in, four story lines, about 13 s, skippable) on start and after every reset.")]
        public bool storyIntro = true;
        [Tooltip("Intro starts in the real room (passthrough) with only the usu, dough and kine; a round portal opens around the usu " +
                 "and the Moon garden spreads out. Off: the intro starts in the garden. Needs storyIntro.")]
        public bool passthroughIntro = true;
        [Tooltip("Seconds for the portal to open.")]
        [Range(1f, 8f)] public float passthroughRevealTime = 3.5f;
        [Tooltip("When the portal opens: as intro line 1-4 appears, or after the last line has gone (default).")]
        public PortalReveal passthroughRevealAt = PortalReveal.AfterLastLine;
        public enum PortalReveal { Line1 = 1, Line2 = 2, Line3 = 3, Line4 = 4, AfterLastLine = 0 }
        [Tooltip("Intro lines written in ink on a hanji handscroll that unrolls once and rolls up after the last line, with " +
                 "gold cloud corners and a small skip hint. Off: the lines float on their own (sky style).")]
        public bool introScroll = true;
        [Range(0.2f, 1f)] public float introScrollOpacity = 0.62f;
        [Header("Intro last line swell")]
        [Tooltip("Slow swell of \"Pound the mochi. Light the Moon.\" once it has risen: peak scale and length up and down (s).")]
        [Range(1f, 1.5f)] public float introSwellScale = 1.2f;
        [Range(0.4f, 4f)] public float introSwellTime = 1.8f;
        [Tooltip("Demo: putting the headset back on after it was off for headsetRestartMinSeconds restarts from the intro.")]
        public bool headsetRestart = true;
        [Header("Story sounds (synthesized unless a clip is set)")]
        [Tooltip("Master switch for StorySounds. Off by default; the MoonProgress sound hooks (onIntroLine, onStrike...) stay.")]
        public bool storySounds = true;
        public bool textSounds = true;
        public bool progressSounds = false;
        [Tooltip("Shimmer on every progress step; turn off when the rhythm game plays its own hit sounds.")]
        public bool strikeSound = true;
        [Range(0f, 1f)] public float textSoundVolume = 0.11f;
        [Range(0f, 1f)] public float progressSfxVolume = 0.3f;
        public AudioClip introLineClip, instructionClip, endingClip, strikeClip, milestoneClip, fullMoonClip, flightClip, arrivalClip, wishClip;
        public float headsetRestartMinSeconds = 3f;

        [Header("Warm light on the dough (lantern side of the usu)")]
        [Tooltip("Small warm Spot light that shines into the bowl, so the dough reads as warm white rice cake under the blue moonlight. " +
                 "Render Mode = ForcePixel (Important): the moonlight takes the one pixel light of Quest's quality level, and an Auto " +
                 "light would fall back to vertex lighting, which barely shows on the dough. No shadows.")]
        public bool mochiWarmLight = true;
        public Color mochiLightColor = new Color(1f, 0.72f, 0.40f);
        [Tooltip("High because Built-in point/spot attenuation is already ~0.2 at the dough (0.73 m from the light, range 1.5 m).")]
        public float mochiLightIntensity = 5.5f;
        [Tooltip("1.5 m reaches the bowl but not the ground below the cone (1.7 m away).")]
        public float mochiLightRange = 1.5f;
        [Tooltip("Cone angle (deg). 0 = Point light instead of Spot.")]
        public float mochiLightSpotAngle = 30f;
        [Tooltip("Position relative to the dough floor (usu on the pedestal, or the practice mortar's hollow). The light aims at the dough. " +
                 "Default: up and to the left front, the side of the left stone lantern.")]
        public Vector3 mochiLightOffset = new Vector3(-0.32f, 0.62f, 0.22f);
        [Tooltip("Layer for MR_Ground and MR_BambooBases, left out of the warm light's culling mask. Their bounds reach up near the bowl, so " +
                 "otherwise Built-in would draw these big meshes again in an additive pass for the light. Default 1 = TransparentFX " +
                 "(only used for lens-flare occlusion, so physics and raycasts are unchanged). -1 = keep Default.")]
        public int warmLightExcludedLayer = 1;

        [Header("Floating island (the sky garden between Earth and Moon)")]
        [Tooltip("On: the ground is a finite island with a rocky underside floating in the starry sky; the sky has stars below " +
                 "the horizon too. Off: the old endless forest floor sinking into the fog.")]
        public bool floatingIsland = true;
        [Tooltip("Island radius (m): a few metres outside the bamboo ring (bambooOuterRadius).")]
        public float islandRadius = 19f;
        [Range(0f, 0.3f)] public float islandEdgeNoise = 0.12f;
        [Tooltip("Depth of the rocky underside below the ground (m).")]
        public float islandDepth = 16f;
        public Color islandRock = Hex(0x3A3F4C);
        [Range(0, 16)] public int floatingRocks = 7;
        [Range(0, 64)] public int skyMistPuffs = 22;
        public Color skyMistColor = new Color(0.55f, 0.6f, 0.86f);

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

        [Header("Usu pedestal (team repo: stand for the team's usu)")]
        [Tooltip("Wooden stump on a stone slab at mortarPosition, part of MR_Environment. The team's usu (MochiStation) stands on its top. " +
                 "Skipped when the XR folder's floor-standing mortar is built (practice project).")]
        public bool pedestal = true;
        [Tooltip("Top of the stump above the ground = height of the usu's base (m). 0.80 puts the dough floor at about 0.83 m and the rim at " +
                 "about 0.98 m: belly height for one-handed strikes while standing, and with the player 0.85 m back, a 1.6 m eye sees " +
                 "the dough about 40 degrees below straight ahead, inside Quest 3's view without tilting the head.")]
        public float pedestalHeight = 0.80f;
        [Tooltip("Radius of the stump top (m). Holds the team's usu (outer radius about 0.21 m) and the kine resting " +
                 "beside it (KineDriver.restPosition x = 0.32 m, head radius 0.06 m).")]
        public float pedestalTopRadius = 0.44f;

        [Header("Team usu/kine skins (MochiPound): sizes must match UsuTool / KineTool / KineDriver")]
        [Tooltip("Generate MR_UsuSkin / MR_KineSkin meshes and materials (World-only installs).")]
        public bool teamUsuKineSkins = true;
        public float usuBase = 0.03f;
        public float usuInnerBottom = 0.125f;
        public float usuInnerTop = 0.15f;
        public float usuDepth = 0.13f;
        public float usuOuterRadius = 0.205f;
        public float kineHeadRadius = 0.06f;
        public float kineHeadLength = 0.20f;
        public float kineFaceRounding = 0.015f;
        [Tooltip("KineDriver.handleLength: the controller (hand) is this far behind the head centre along local -Z.")]
        public float kineHandleLength = 0.55f;

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
