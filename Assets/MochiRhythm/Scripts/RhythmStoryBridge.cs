using MoonRabbit.World;
using UnityEngine;

namespace MochiRhythm
{
    /// <summary>
    /// Connects a rhythm song to the Moon story: intro -> pick a song -> the team's play fills the Moon ->
    /// full Moon plays the story's ending (mochi to Earth, "Happy Chuseok, Earth!") -> the story resets.
    ///  - While a song runs, the story's demo strike counter is off and the Moon shows
    ///    TeamCredit / (targetFraction x notes in one play-through). TeamCredit gives each note up to 1:
    ///    the pounder's strike (Perfect 1, Good 0.6) plus a dough turn before it (0.5), capped at 1; with the
    ///    Partner pounding, a turn alone is worth 1. The combo multiplier does not touch the Moon, so one miss
    ///    does not stall it.
    ///  - The song loops; after maxPasses play-throughs the ending plays anyway, so nobody is stuck.
    ///  - When the ending starts the song fades out and the game stops. Free play (no song) keeps the demo counter.
    ///  - Songs cannot start during the intro or the ending (see <see cref="WaitReason"/>).
    /// For two players, only MoonProgress's value needs syncing.
    /// </summary>
    public class RhythmStoryBridge : MonoBehaviour
    {
        public RhythmMode mode;
        [Tooltip("Share of a perfect play-through (in team credit) that fills the Moon.")]
        [Range(0.2f, 1f)] public float targetFraction = 0.7f;
        [Tooltip("Play-throughs of the song before the ending plays anyway.")]
        [Min(1)] public int maxPasses = 2;
        [Tooltip("Seconds the song takes to fade out when the ending starts.")]
        public float musicFade = 2.5f;

        bool wasActive, demoBefore = true, haveDemoBefore;
        float lastSent = -1f;

        /// <summary>Why a song cannot start now (intro or ending playing), or null.</summary>
        public static string WaitReason
        {
            get
            {
                var mp = MoonProgress.Instance;
                if (mp == null) return null;
                if (mp.IntroActive) return "Songs start after the story's intro";
                if (mp.IsComplete || mp.Locked) return "Wait for the ending to finish";
                return null;
            }
        }

        void Update()
        {
            if (mode == null || mode.Game == null || mode.Music == null) return;
            var mp = MoonProgress.Instance;
            var g = mode.Game;
            bool active = g.Active;
            if (active && !wasActive) OnSongStart(mp);
            if (!active && wasActive) OnSongEnd(mp);
            wasActive = active;
            KneadGhostGuide.Suppressed = active;
            if (!active || mp == null) return;

            // the ending has started (full Moon): the song bows out
            if (mp.IsComplete || mp.Locked)
            {
                Debug.Log("[rhythm] full Moon: song fades out, the story's ending plays");
                mode.Music.FadeOutAndStop(musicFade);
                g.StopGame();
                return;
            }

            float target = Mathf.Max(1f, targetFraction * g.NotesPerPass);
            float p = g.TeamCredit / target;
            double played = g.SongDuration > 0 ? mode.Music.SongTime() / g.SongDuration : 0;
            if (played >= maxPasses && p < 1f)
            {
                Debug.Log($"[rhythm] {maxPasses} play-throughs at {p:P0} of the target: the ending plays anyway");
                p = 1f;
            }
            if (Mathf.Abs(p - lastSent) > 1e-4f)
            {
                mp.Set(p);
                lastSent = p;
            }
        }

        void OnSongStart(MoonProgress mp)
        {
            lastSent = -1f;
            if (mp == null) return;
            if (!haveDemoBefore) { demoBefore = mp.demoStrikeDetection; haveDemoBefore = true; }
            mp.demoStrikeDetection = false;
            if (mp.Value > 0f && !mp.Locked && !mp.IsComplete) mp.ResetProgress(); // the song fills the Moon from a crescent
            int notes = mode.Game.NotesPerPass;
            Debug.Log($"[rhythm] song started: the Moon fills at {targetFraction * notes:0} team credit ({notes} notes per play-through)");
        }

        void OnSongEnd(MoonProgress mp)
        {
            KneadGhostGuide.Suppressed = false;
            if (mp != null && haveDemoBefore) mp.demoStrikeDetection = demoBefore;
            haveDemoBefore = false;
        }

        void OnDisable()
        {
            if (wasActive) OnSongEnd(MoonProgress.Instance);
            wasActive = false;
        }
    }
}
