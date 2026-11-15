using System;
using Satie.Core;
using UnityEngine;

namespace MochiRhythm
{
    /// <summary>
    /// Starts Satie in StereoMix output when the spatializer the project names is not actually available.
    ///
    /// Satie's component (Packages/world.satie.unity) starts by itself in EngineSpatializer output: one
    /// AudioSource per voice, with spatialize = true whenever ProjectSettings/AudioManager names a spatializer
    /// plugin. This project names "Meta XR Audio" but does not install the Meta XR Audio SDK, and a source with
    /// spatialize = true and no loaded plugin plays silence on Quest (the same problem StorySounds checks for).
    /// So before the first scene loads, if the named plugin is not among the loaded ones, this starts Satie
    /// itself with StereoMix (the core's own panned stereo mix on one AudioSource); Satie's own boot then sees
    /// an instance and stays out. With a working spatializer, nothing changes and Satie starts as usual.
    /// Define MOCHI_SATIE_DEFAULT_START to leave Satie's start alone.
    /// </summary>
    public static class SatieStartup
    {
#if !MOCHI_SATIE_DEFAULT_START
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        static void Start()
        {
            if (SatieCoreBehaviour.Instance != null) return;
            if (Resources.Load<TextAsset>("SatieCore/scene.manifest") == null) return; // no compiled scene: Satie stays off anyway
            string named = AudioSettings.GetSpatializerPluginName() ?? "";
            if (named.Length == 0 || SpatializerLoaded(named)) return; // Satie's default output works here
            var go = new GameObject("Satie");
            go.SetActive(false); // so Awake runs after the output is chosen
            var satie = go.AddComponent<SatieCoreBehaviour>();
            satie.output = SatieOutput.StereoMix;
            UnityEngine.Object.DontDestroyOnLoad(go);
            go.SetActive(true);
            Debug.Log($"[sound] spatializer '{named}' is named in the project but not loaded: Satie plays its own stereo mix");
        }
#endif

        static bool SpatializerLoaded(string name)
        {
#if UNITY_EDITOR
            // GetSpatializerPluginNames is Editor-only. A player cannot tell whether the named plugin is really
            // loaded, so it reports "not loaded" and Satie plays its own stereo mix (never silent).
            try
            {
                var loaded = AudioSettings.GetSpatializerPluginNames();
                if (loaded == null) return false;
                foreach (var n in loaded) if (n == name) return true;
            }
            catch (Exception) { }
#endif
            return false;
        }
    }
}
