// SatieCoreDemo.cs: a scene you can hear without building one. On Play it reads the
// compiled scene's manifest, makes one marker object per host where the manifest
// puts it, a camera with the ears, and the SatieCoreBehaviour. It draws nothing:
// the scene is played and mixed from the Satie mixer (Window > Satie > Mixer), and
// the ears are the camera, moved like any object. Add it to an empty object in a
// scene that has no SatieCoreBehaviour of its own; a game never needs it (the
// behaviour starts by itself when a compiled scene is in Resources).
using System.Collections.Generic;
using UnityEngine;

namespace Satie.Core
{
    public class SatieCoreDemo : MonoBehaviour
    {
        SatieManifest manifest;
        SatieCoreBehaviour core;
        Transform ears;
        bool engineSpatializer = true;

        public static SatieCoreDemo Instance { get; private set; }

        void Awake()
        {
            manifest = JsonUtility.FromJson<SatieManifest>(Resources.Load<TextAsset>("SatieCore/scene.manifest").text);
            // Hosts first, so the behaviour finds them by name when it wakes.
            foreach (SatieHostEntry h in manifest.hostList)
            {
                GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                marker.name = string.IsNullOrEmpty(h.@object) ? h.id : h.@object;
                marker.transform.localScale = Vector3.one * 0.3f;
                if (h.hasPosition) marker.transform.position = new Vector3(h.position[0], h.position[1], h.position[2]);
            }
            Camera camera = Camera.main;
            if (camera == null)
            {
                GameObject go = new GameObject("Ears");
                camera = go.AddComponent<Camera>();
                go.AddComponent<AudioListener>();
            }
            ears = camera.transform;
            ears.position = new Vector3(0f, 1.6f, 0f);
            Instance = this;
            Build();
        }

        void Build()
        {
            // What the old core was sent, so the new one starts where it left off.
            Dictionary<string, float> signals = new Dictionary<string, float>();
            string state = null;
            if (core != null)
            {
                foreach (SatieSignalEntry s in manifest.signalList) signals[s.name] = core.SignalValue(s.name);
                state = core.CurrentState;
            }
            // At once, not at the end of the frame: the next behaviour must find no other instance alive.
            if (core != null) DestroyImmediate(core.gameObject);
            GameObject go = new GameObject("Satie");
            go.SetActive(false);
            core = go.AddComponent<SatieCoreBehaviour>();
            core.output = engineSpatializer ? SatieOutput.EngineSpatializer : SatieOutput.StereoMix;
            core.listener = ears;
            go.SetActive(true);
            foreach (KeyValuePair<string, float> pair in signals) core.Signal(pair.Key, pair.Value);
            if (!string.IsNullOrEmpty(state)) core.State(state);
        }

        /// True while Unity gives each sound its direction; false while the core's own stereo mix plays.
        public bool DirectionByUnity { get { return engineSpatializer; } }

        /// Swap who gives each sound its direction. The scene starts again on a new core, with the same signals and state.
        public void SwitchDirection()
        {
            engineSpatializer = !engineSpatializer;
            Build();
        }
    }
}
