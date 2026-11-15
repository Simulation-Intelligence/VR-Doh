using UnityEngine;
using UnityEngine.SceneManagement;

namespace MochiRhythm
{
    /// <summary>
    /// Adds rhythm mode at runtime to any loaded scene that has a MochiStation using tracked hands
    /// (MochiStandalone, MochiPounding), so no scene file is edited. Rhythm mode stays idle until someone
    /// presses Start on its panel. Define MOCHI_RHYTHM_OFF (Player Settings > Scripting Define Symbols)
    /// to switch the automatic attach off; RhythmModeBootstrap.Attach() still adds it on demand.
    /// </summary>
    public static class RhythmModeBootstrap
    {
#if !MOCHI_RHYTHM_OFF
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        static void Install()
        {
            SceneManager.sceneLoaded -= OnSceneLoaded; // safe with domain reload off
            SceneManager.sceneLoaded += OnSceneLoaded;
            Attach();
        }

        static void OnSceneLoaded(Scene scene, LoadSceneMode loadMode) => Attach();
#endif

        public static RhythmMode Attach()
        {
            var existing = Object.FindAnyObjectByType<RhythmMode>();
            if (existing != null) return existing;
            var station = Object.FindAnyObjectByType<MochiStation>();
            if (station == null || station.toolSet != MochiStation.ToolSet.TrackedHands) return null;
            var go = new GameObject("MochiRhythm");
            go.SetActive(false);
            var mode = go.AddComponent<RhythmMode>();
            mode.station = station;
            go.SetActive(true);
            return mode;
        }
    }
}
