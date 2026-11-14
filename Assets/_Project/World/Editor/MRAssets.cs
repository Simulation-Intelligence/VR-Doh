using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using Object = UnityEngine.Object;

namespace MoonRabbit.WorldEditor
{
    /// <summary>
    /// Asset helpers. Every helper overwrites an existing asset IN PLACE, so GUIDs
    /// (and the Builder's references to prefabs, meshes and materials) never change.
    /// </summary>
    public static class MRAssets
    {
        public const string WorldRoot = "Assets/_Project/World";
        public const string XRRoot = "Assets/_Project/XR";
        public const string WorldGenerated = WorldRoot + "/Generated";
        public const string XRGenerated = XRRoot + "/Generated";

        public static void EnsureFolder(string path)
        {
            if (AssetDatabase.IsValidFolder(path)) return;
            var parent = Path.GetDirectoryName(path)?.Replace('\\', '/');
            if (!string.IsNullOrEmpty(parent)) EnsureFolder(parent);
            AssetDatabase.CreateFolder(parent, Path.GetFileName(path));
        }

        public static Mesh SaveMesh(Mesh mesh, string path)
        {
            EnsureFolder(Path.GetDirectoryName(path)?.Replace('\\', '/'));
            var existing = AssetDatabase.LoadAssetAtPath<Mesh>(path);
            if (existing != null)
            {
                existing.Clear();
                EditorUtility.CopySerialized(mesh, existing);
                existing.name = Path.GetFileNameWithoutExtension(path);
                EditorUtility.SetDirty(existing);
                Object.DestroyImmediate(mesh);
                return existing;
            }
            mesh.name = Path.GetFileNameWithoutExtension(path);
            AssetDatabase.CreateAsset(mesh, path);
            return mesh;
        }

        /// <summary>True when the project renders with the Built-in pipeline (no SRP asset assigned).
        /// The team repo (MochiPound) is Built-in with the URP package installed but unused,
        /// so "URP/Lit can be found" does NOT mean URP is active.</summary>
        public static bool IsBuiltInPipeline => GraphicsSettings.currentRenderPipeline == null;

        /// <summary>Standard on the Built-in pipeline, URP/Lit when URP is the active pipeline.</summary>
        public static Shader LitShader
        {
            get
            {
                if (IsBuiltInPipeline) return Shader.Find("Standard");
                return Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            }
        }

        public static Material GetMaterial(string path, Shader shader)
        {
            EnsureFolder(Path.GetDirectoryName(path)?.Replace('\\', '/'));
            var m = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (m == null)
            {
                m = new Material(shader);
                AssetDatabase.CreateAsset(m, path);
            }
            else if (m.shader != shader)
            {
                m.shader = shader;                     // e.g. URP/Lit -> Standard after a pipeline change
                m.shaderKeywords = Array.Empty<string>(); // drop the old shader's keywords; callers set them again
            }
            return m;
        }

        /// <summary>Lit material (URP/Lit or Built-in Standard), matte, instancing on, optional emission.
        /// Rebuilding after a pipeline change switches the shader in place (same GUID).</summary>
        public static Material Lit(string folder, string name, Color color, float smoothness = 0.12f, Color? emission = null)
        {
            var m = GetMaterial($"{folder}/Materials/{name}.mat", LitShader);
            if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", color);
            if (m.HasProperty("_Color")) m.SetColor("_Color", color);
            if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", smoothness);
            if (m.HasProperty("_Glossiness")) m.SetFloat("_Glossiness", smoothness);
            if (m.HasProperty("_Metallic")) m.SetFloat("_Metallic", 0f);
            if (m.HasProperty("_SpecularHighlights")) m.SetFloat("_SpecularHighlights", 0f);
            if (m.HasProperty("_EnvironmentReflections")) m.SetFloat("_EnvironmentReflections", 0f); // URP
            if (m.HasProperty("_GlossyReflections")) m.SetFloat("_GlossyReflections", 0f);           // Standard
            m.EnableKeyword("_SPECULARHIGHLIGHTS_OFF");
            m.EnableKeyword("_ENVIRONMENTREFLECTIONS_OFF");
            m.EnableKeyword("_GLOSSYREFLECTIONS_OFF");

            if (emission.HasValue && emission.Value.maxColorComponent > 0f)
            {
                m.EnableKeyword("_EMISSION");
                m.SetColor("_EmissionColor", emission.Value);
                m.globalIlluminationFlags = MaterialGlobalIlluminationFlags.None;
            }
            else
            {
                m.DisableKeyword("_EMISSION");
                if (m.HasProperty("_EmissionColor")) m.SetColor("_EmissionColor", Color.black);
            }
            m.enableInstancing = true;
            EditorUtility.SetDirty(m);
            return m;
        }

        /// <summary>Writes a PNG next to the project and imports it with the given settings.</summary>
        public static Texture2D SavePng(Texture2D tex, string path, bool mipmaps, TextureWrapMode wrapU, TextureWrapMode wrapV, int maxSize)
        {
            EnsureFolder(Path.GetDirectoryName(path)?.Replace('\\', '/'));
            var full = Path.Combine(Directory.GetParent(Application.dataPath).FullName, path);
            File.WriteAllBytes(full, tex.EncodeToPNG());
            Object.DestroyImmediate(tex);
            AssetDatabase.ImportAsset(path, ImportAssetOptions.ForceUpdate);
            var imp = (TextureImporter)AssetImporter.GetAtPath(path);
            imp.textureType = TextureImporterType.Default;
            imp.mipmapEnabled = mipmaps;
            imp.wrapModeU = wrapU;
            imp.wrapModeV = wrapV;
            imp.maxTextureSize = maxSize;
            imp.textureCompression = TextureImporterCompression.Compressed;
            imp.sRGBTexture = true;
            imp.alphaSource = TextureImporterAlphaSource.None;
            imp.SaveAndReimport();
            return AssetDatabase.LoadAssetAtPath<Texture2D>(path);
        }

        /// <summary>Saves a hierarchy built in a preview scene as a prefab (keeps the prefab GUID).</summary>
        public static GameObject SavePrefab(GameObject root, string path)
        {
            EnsureFolder(Path.GetDirectoryName(path)?.Replace('\\', '/'));
            var prefab = PrefabUtility.SaveAsPrefabAsset(root, path, out bool ok);
            if (!ok) Debug.LogError($"[Moon Rabbit] Could not save prefab {path}");
            return prefab;
        }

        /// <summary>Adds a tag to the TagManager if missing. ProjectSettings belong to the Builder, so we warn.</summary>
        public static bool EnsureTag(string tag)
        {
            if (UnityEditorInternal.InternalEditorUtility.tags.Contains(tag)) return true;
            var assets = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset");
            if (assets == null || assets.Length == 0) return false;
            var so = new SerializedObject(assets[0]);
            var tags = so.FindProperty("tags");
            tags.InsertArrayElementAtIndex(tags.arraySize);
            tags.GetArrayElementAtIndex(tags.arraySize - 1).stringValue = tag;
            so.ApplyModifiedPropertiesWithoutUndo();
            Debug.LogWarning($"[Moon Rabbit] Added the tag \"{tag}\" to ProjectSettings/TagManager.asset on THIS machine. " +
                             "ProjectSettings belong to Nasif: do not commit TagManager.asset; ask Nasif to add the tag " +
                             "(StrikeZone detection also works without it, through the StrikeZone component).");
            return true;
        }

        // ---------- reflection helpers (so the editor code compiles on XRI 2.5 and 3.x) ----------

        public static Type FindType(params string[] fullNames)
        {
            // TypeCache instead of AppDomain.GetAssemblies() (Unity 6.6 analyzer UAC0005). Every type we look up is a Component.
            var components = TypeCache.GetTypesDerivedFrom<Component>();
            foreach (var name in fullNames)
                foreach (var t in components)
                    if (t.FullName == name) return t;
            return null;
        }

        public static void SetEnum(SerializedObject so, string prop, string valueName)
        {
            var p = so.FindProperty(prop);
            if (p == null) { Debug.LogWarning($"[Moon Rabbit] {so.targetObject.GetType().Name}.{prop} not found"); return; }
            string Norm(string s) => s.Replace(" ", "").ToLowerInvariant();
            for (int i = 0; i < p.enumNames.Length; i++)
            {
                if (Norm(p.enumNames[i]) == Norm(valueName))
                {
                    p.enumValueIndex = i;
                    return;
                }
            }
            Debug.LogWarning($"[Moon Rabbit] Enum value {valueName} not found on {prop}");
        }

        public static void Set(SerializedObject so, string prop, object value)
        {
            var p = so.FindProperty(prop);
            if (p == null) { Debug.LogWarning($"[Moon Rabbit] {so.targetObject.GetType().Name}.{prop} not found"); return; }
            switch (value)
            {
                case bool b: p.boolValue = b; break;
                case float f: p.floatValue = f; break;
                case int i: p.intValue = i; break;
                case Object o: p.objectReferenceValue = o; break;
                default: Debug.LogWarning($"[Moon Rabbit] Unsupported value for {prop}"); break;
            }
        }
    }
}
