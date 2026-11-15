#if MR_TMP
using TMPro;
using UnityEditor;
using UnityEngine;
using UnityEngine.TextCore.LowLevel;

namespace MoonRabbit.WorldEditor
{
    /// <summary>
    /// TextMesh Pro SDF font assets and materials for the story text (World/Fonts, SIL OFL 1.1, from github.com/google/fonts):
    /// intro lines in Cinzel SemiBold, the ending line in Baloo 2 SemiBold (static wght 600 instances of the variable fonts,
    /// so strokes are not thin in VR). Static atlases, 1024², ASCII + "…", generous padding for outline/underlay/glow.
    /// Rebuilt by Build Environment; the assets keep their GUIDs.
    /// </summary>
    public static class StoryFontBuilder
    {
        const string F = MRAssets.WorldRoot + "/Fonts";
        public const string IntroFontPath = F + "/Cinzel/MR_Cinzel SDF.asset";
        public const string EndingFontPath = F + "/Baloo2/MR_Baloo2 SDF.asset";
        public const string IntroMatPath = F + "/Cinzel/MR_IntroText.mat";
        public const string EndingMatPath = F + "/Baloo2/MR_EndingText.mat";
        public const string InstructionFontPath = F + "/Cinzel/MR_Cinzel Black SDF.asset";
        public const string InstructionMatPath = F + "/Cinzel/MR_InstructionText.mat";

        public class Fonts
        {
            public TMP_FontAsset intro, ending, instruction;
            public Material introMat, endingMat, instructionMat;
        }

        [MenuItem("Moon Rabbit/Rebuild Story Fonts", false, 30)]
        public static void RebuildMenu() => Build(true);

        /// <summary>Story fonts and their materials; nulls where a TTF is missing.</summary>
        public static Fonts Build(bool force = false)
        {
            var r = new Fonts
            {
                intro = FontAsset(F + "/Cinzel/Cinzel-SemiBold.ttf", IntroFontPath, force),
                ending = FontAsset(F + "/Baloo2/Baloo2-Bold.ttf", EndingFontPath, force || NeedsSource(EndingFontPath, "Baloo 2 Bold")),
                instruction = FontAsset(F + "/Cinzel/Cinzel-Black.ttf", InstructionFontPath, force), // "instruction" style: Cinzel Black
            };
            if (r.intro != null) r.introMat = TextMaterial(r.intro, IntroMatPath, 0.16f, new Color(1f, 0.8f, 0.55f, 0.35f));
            if (r.ending != null) r.endingMat = TextMaterial(r.ending, EndingMatPath, 0.16f, new Color(1f, 0.76f, 0.38f, 0.6f), 0.85f); // wide soft gold glow
            if (r.instruction != null)
            {
                r.instructionMat = TextMaterial(r.instruction, InstructionMatPath, 0.2f, new Color(1f, 0.78f, 0.35f, 0.6f), 0.7f);
                // inside a narration line "<b>" switches to Cinzel Black, which then draws with its own default material:
                // give that material the same outline / shadow / gold glow
                Style(r.instruction.material, r.instruction, 0.2f, new Color(1f, 0.78f, 0.35f, 0.6f), 0.7f);
                EditorUtility.SetDirty(r.instruction.material);
                if (r.intro != null)
                {
                    var table = r.intro.fontWeightTable;
                    if (table != null && table.Length > 7)
                    {
                        table[7].regularTypeface = r.instruction; // weight 700 (<b>) = Cinzel Black (the array is the asset's own)
                        EditorUtility.SetDirty(r.intro);
                    }
                }
            }
            AssetDatabase.SaveAssets();
            return r;
        }

        /// <summary>True when the font asset was made from another TTF (e.g. SemiBold before Bold was chosen).</summary>
        static bool NeedsSource(string assetPath, string familyName)
        {
            var fa = AssetDatabase.LoadAssetAtPath<TMP_FontAsset>(assetPath);
            // static atlases drop their source file reference, so compare the family name baked into the asset
            return fa != null && fa.faceInfo.familyName != familyName;
        }

        static TMP_FontAsset FontAsset(string ttfPath, string assetPath, bool force)
        {
            var existing = AssetDatabase.LoadAssetAtPath<TMP_FontAsset>(assetPath);
            if (existing != null && !force && existing.characterTable.Count > 90) return existing;
            var font = AssetDatabase.LoadAssetAtPath<Font>(ttfPath);
            if (font == null) { Debug.LogWarning("[Moon Rabbit] Story font missing: " + ttfPath); return existing; }

            // SDF with a wide spread (padding 12 at 90 pt) so outline, underlay and glow stay smooth.
            var fa = TMP_FontAsset.CreateFontAsset(font, 90, 12, GlyphRenderMode.SDFAA, 1024, 1024, AtlasPopulationMode.Dynamic, true);
            string chars = "";
            for (char c = (char)32; c < 127; c++) chars += c;
            chars += "…’‘“”—·";
            fa.TryAddCharacters(chars, out string missing);
            if (!string.IsNullOrEmpty(missing)) Debug.Log("[Moon Rabbit] " + font.name + ": no glyphs for '" + missing + "'.");
            fa.atlasPopulationMode = AtlasPopulationMode.Static;
            fa.name = System.IO.Path.GetFileNameWithoutExtension(assetPath);

            if (existing != null)
            {
                // keep the GUID: copy into the existing asset and replace its sub-assets
                foreach (var sub in AssetDatabase.LoadAllAssetRepresentationsAtPath(assetPath)) Object.DestroyImmediate(sub, true);
                EditorUtility.CopySerialized(fa, existing);
                existing.name = fa.name;
                var tex = fa.atlasTexture; tex.name = fa.name + " Atlas";
                var mat = fa.material; mat.name = fa.name + " Material";
                AssetDatabase.AddObjectToAsset(tex, existing);
                AssetDatabase.AddObjectToAsset(mat, existing);
                existing.atlasTextures = new[] { tex };
                existing.material = mat;
                mat.SetTexture(ShaderUtilities.ID_MainTex, tex);
                EditorUtility.SetDirty(existing);
                // the temporary asset must not take the (now saved) atlas and material with it when it is destroyed
                fa.atlasTextures = new Texture2D[0];
                fa.material = null;
                Object.DestroyImmediate(fa);
                return existing;
            }
            AssetDatabase.CreateAsset(fa, assetPath);
            fa.atlasTexture.name = fa.name + " Atlas";
            fa.material.name = fa.name + " Material";
            AssetDatabase.AddObjectToAsset(fa.atlasTexture, fa);
            AssetDatabase.AddObjectToAsset(fa.material, fa);
            EditorUtility.SetDirty(fa);
            return fa;
        }

        /// <summary>Outline + soft drop shadow (underlay) + warm outer glow, on top of the font's atlas.</summary>
        static Material TextMaterial(TMP_FontAsset fa, string path, float outline, Color glow, float glowOuter = 0.55f)
        {
            var m = MRAssets.GetMaterial(path, fa.material.shader);
            m.CopyPropertiesFromMaterial(fa.material);
            Style(m, fa, outline, glow, glowOuter);
            return m;
        }

        static void Style(Material m, TMP_FontAsset fa, float outline, Color glow, float glowOuter)
        {
            m.SetTexture(ShaderUtilities.ID_MainTex, fa.atlasTexture);
            m.SetFloat(ShaderUtilities.ID_OutlineWidth, outline);
            m.SetColor(ShaderUtilities.ID_OutlineColor, new Color(0.08f, 0.07f, 0.12f, 0.85f));
            m.EnableKeyword(ShaderUtilities.Keyword_Outline);
            m.EnableKeyword(ShaderUtilities.Keyword_Underlay);
            m.SetColor(ShaderUtilities.ID_UnderlayColor, new Color(0f, 0f, 0f, 0.6f));
            m.SetFloat(ShaderUtilities.ID_UnderlayOffsetX, 0.6f);
            m.SetFloat(ShaderUtilities.ID_UnderlayOffsetY, -0.6f);
            m.SetFloat(ShaderUtilities.ID_UnderlaySoftness, 0.45f);
            m.SetFloat(ShaderUtilities.ID_UnderlayDilate, 0.15f);
            m.EnableKeyword(ShaderUtilities.Keyword_Glow);
            m.SetColor(ShaderUtilities.ID_GlowColor, glow);
            m.SetFloat(ShaderUtilities.ID_GlowOuter, glowOuter);
            m.SetFloat(ShaderUtilities.ID_GlowPower, 0.6f);
            m.SetFloat(ShaderUtilities.ID_FaceDilate, 0.05f);
            EditorUtility.SetDirty(m);
        }
    }
}
#endif
