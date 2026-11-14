using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

namespace MoonRabbit.WorldEditor
{
    /// <summary>
    /// Menu: Moon Rabbit > Build Environment
    ///   1. opens (or creates) Jihyeon's sandbox scene Assets/_Project/World/Sandbox/MR_Sandbox.unity (git-ignored),
    ///   2. regenerates meshes, materials, textures and the three prefabs,
    ///   3. places environment + mortar + pestle + XR rig + XR Device Simulator in the sandbox.
    /// Prefabs keep their GUIDs on every rebuild, so the Builder's scenes keep working.
    /// The World folder does not depend on XR: the mortar/pestle builder lives in Assets/_Project/XR/Editor
    /// and registers itself in <see cref="PropBuilder"/>. Without the XR folder (install_kit.sh --world-only),
    /// only the environment is built and the sandbox gets a plain preview camera.
    /// </summary>
    public static class MoonRabbitMenu
    {
        public struct SandboxProp
        {
            public GameObject prefab;
            public Vector3 position;
        }

        /// <summary>Set by MoonRabbit.XR.Editor (MortarPestleBuilder). Returns the props to place, or null on failure.</summary>
        public static Func<MoonRabbitEnvironmentSettings, Scene, List<SandboxProp>> PropBuilder;

        public const string SettingsPath = MRAssets.WorldRoot + "/Editor/MR_EnvironmentSettings.asset";
        public const string SandboxPath = MRAssets.WorldRoot + "/Sandbox/MR_Sandbox.unity";

        [MenuItem("Moon Rabbit/Build Environment", false, 0)]
        public static void BuildEnvironment()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            var s = GetSettings();
            OpenOrCreateSandbox();
            if (!BuildPrefabs(s, out var env, out var props)) return;
            PlaceInSandbox(s, env, props);
            LogBuilt(env, props);
        }

        [MenuItem("Moon Rabbit/Rebuild Prefabs Only (no scene changes)", false, 1)]
        public static void RebuildPrefabsOnly()
        {
            var s = GetSettings();
            if (BuildPrefabs(s, out _, out _))
                Debug.Log("[Moon Rabbit] Prefabs rebuilt (no scene was changed).");
        }

        [MenuItem("Moon Rabbit/Select Build Settings", false, 20)]
        public static void SelectSettings()
        {
            Selection.activeObject = GetSettings();
            EditorGUIUtility.PingObject(Selection.activeObject);
        }

        /// <summary>Batch-mode entry: Unity -batchmode -executeMethod MoonRabbit.WorldEditor.MoonRabbitMenu.BuildFromCommandLine</summary>
        public static void BuildFromCommandLine()
        {
            var s = GetSettings();
            OpenOrCreateSandbox();
            if (!BuildPrefabs(s, out var env, out var props))
                throw new Exception("Moon Rabbit build failed");
            PlaceInSandbox(s, env, props);
            LogBuilt(env, props);
        }

        static void LogBuilt(GameObject env, List<SandboxProp> props)
        {
            var paths = new List<string> { AssetDatabase.GetAssetPath(env) };
            foreach (var p in props) paths.Add(AssetDatabase.GetAssetPath(p.prefab));
            Debug.Log("[Moon Rabbit] Environment built (" + (MRAssets.IsBuiltInPipeline ? "Built-in" : "URP") + " pipeline). Prefabs: " +
                      string.Join(", ", paths) +
                      (PropBuilder == null ? ". World-only: no XR folder, so no mortar/pestle." : "."));
        }

        // ------------------------------------------------------------------

        public static MoonRabbitEnvironmentSettings GetSettings()
        {
            var s = AssetDatabase.LoadAssetAtPath<MoonRabbitEnvironmentSettings>(SettingsPath);
            if (s != null) return s;
            MRAssets.EnsureFolder(Path.GetDirectoryName(SettingsPath)?.Replace('\\', '/'));
            s = ScriptableObject.CreateInstance<MoonRabbitEnvironmentSettings>();
            AssetDatabase.CreateAsset(s, SettingsPath);
            AssetDatabase.SaveAssets();
            return s;
        }

        static bool BuildPrefabs(MoonRabbitEnvironmentSettings s, out GameObject env, out List<SandboxProp> props)
        {
            env = null;
            props = new List<SandboxProp>();
            var scene = EditorSceneManager.NewPreviewScene();
            try
            {
                EditorUtility.DisplayProgressBar("Moon Rabbit", "Environment (bamboo, cassia tree, lanterns, sky)...", 0.2f);
                env = EnvironmentBuilder.BuildPrefab(s, scene);
                if (PropBuilder != null)
                {
                    EditorUtility.DisplayProgressBar("Moon Rabbit", "Mortar and pestle...", 0.75f);
                    props = PropBuilder(s, scene);
                }
                AssetDatabase.SaveAssets();
                return env != null && props != null;
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                return false;
            }
            finally
            {
                EditorSceneManager.ClosePreviewScene(scene);
                EditorUtility.ClearProgressBar();
            }
        }

        static void OpenOrCreateSandbox()
        {
            MRAssets.EnsureFolder(Path.GetDirectoryName(SandboxPath)?.Replace('\\', '/'));
            if (File.Exists(SandboxPath))
            {
                EditorSceneManager.OpenScene(SandboxPath, OpenSceneMode.Single);
            }
            else
            {
                var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
                EditorSceneManager.SaveScene(scene, SandboxPath);
            }
        }

        static void PlaceInSandbox(MoonRabbitEnvironmentSettings s, GameObject env, List<SandboxProp> props)
        {
            var scene = SceneManager.GetActiveScene();
            foreach (var go in scene.GetRootGameObjects())
                if (go.name.StartsWith("MR_")) Object.DestroyImmediate(go);

            var envInst = (GameObject)PrefabUtility.InstantiatePrefab(env, scene);
            envInst.transform.position = Vector3.zero;
            GameObject select = envInst;
            foreach (var p in props)
            {
                var inst = (GameObject)PrefabUtility.InstantiatePrefab(p.prefab, scene);
                inst.transform.position = p.position;
                if (select == envInst) select = inst; // first prop (the mortar)
            }

            EnsureRig(s, scene);

            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene);
            Selection.activeGameObject = select;

            var view = SceneView.lastActiveSceneView;
            if (view != null)
                view.LookAt(s.mortarPosition + Vector3.up * 1.2f, Quaternion.Euler(12f, 0f, 0f), 7f);
        }

        static void EnsureRig(MoonRabbitEnvironmentSettings s, Scene scene)
        {
            var roots = scene.GetRootGameObjects();
            // No XR Interaction Toolkit (e.g. the team repo uses the Meta XR SDK): a plain camera is the
            // expected setup, not a problem, so no warnings.
            bool hasXri = MRAssets.FindType("Unity.XR.CoreUtils.XROrigin") != null &&
                          MRAssets.FindType("UnityEngine.XR.Interaction.Toolkit.Interactables.XRGrabInteractable",
                                            "UnityEngine.XR.Interaction.Toolkit.XRGrabInteractable") != null;
            if (!hasXri)
            {
                if (!roots.Any(r => r.name == "MR_PreviewCamera" || r.GetComponentInChildren<Camera>() != null))
                    AddPreviewCamera(s, scene);
                return;
            }
            bool hasRig = roots.Any(r => r.name.Contains("XR Origin") || r.name.Contains("XR Rig"));
            bool hasSim = roots.Any(r => r.name.Contains("Simulator"));

            if (!hasRig)
            {
                var rig = FindPrefab("Complete XR Origin Set Up", "XR Origin (XR Rig)");
                if (rig != null)
                {
                    var inst = (GameObject)PrefabUtility.InstantiatePrefab(rig, scene);
                    PlaceRig(inst, s.playerPosition);
                }
                else
                {
                    Debug.LogWarning("[Moon Rabbit] XR Interaction Toolkit 'Starter Assets' sample not found " +
                                     "(Package Manager > XR Interaction Toolkit > Samples > Starter Assets). Added a plain camera instead.");
                    AddPreviewCamera(s, scene);
                }
            }

            if (!hasSim)
            {
                var sim = FindPrefab("XR Interaction Simulator", "XR Device Simulator"); // XRI 3.x first, 2.x fallback
                if (sim != null) PrefabUtility.InstantiatePrefab(sim, scene);
                else Debug.LogWarning("[Moon Rabbit] 'XR Interaction Simulator' sample not found " +
                                      "(Package Manager > XR Interaction Toolkit > Samples > XR Interaction Simulator).");
            }
        }

        static void AddPreviewCamera(MoonRabbitEnvironmentSettings s, Scene scene)
        {
            var cam = new GameObject("MR_PreviewCamera", typeof(Camera), typeof(AudioListener));
            SceneManager.MoveGameObjectToScene(cam, scene);
            cam.tag = "MainCamera";
            cam.GetComponent<Camera>().farClipPlane = 1000f; // the Moon is ~140 m away
            cam.transform.position = s.playerPosition + new Vector3(0f, 1.6f, -1.6f);
            cam.transform.LookAt(s.mortarPosition + Vector3.up * 0.8f);
        }

        static void PlaceRig(GameObject rig, Vector3 position)
        {
            rig.transform.position = position;
            rig.transform.rotation = Quaternion.identity; // face +Z: mortar, cassia tree and Moon ahead
            // "Complete XR Origin Set Up" wraps the actual XR Origin in a parent; that's fine either way.
        }

        static GameObject FindPrefab(params string[] names)
        {
            var guids = AssetDatabase.FindAssets("t:Prefab");
            foreach (var name in names)
                foreach (var g in guids)
                {
                    var path = AssetDatabase.GUIDToAssetPath(g);
                    if (Path.GetFileNameWithoutExtension(path) == name)
                        return AssetDatabase.LoadAssetAtPath<GameObject>(path);
                }
            return null;
        }
    }
}
