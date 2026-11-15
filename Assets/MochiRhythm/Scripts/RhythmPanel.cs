using System;
using TMPro;
using UnityEngine;
using UnityEngine.InputSystem;

namespace MochiRhythm
{
    /// <summary>
    /// Rhythm-mode controls on a small board lying on the table to the kneader's left (a child of the
    /// station, so it moves with it), plus the score floating beyond the usu while a game runs.
    /// Keyboard, for the Editor and the Meta XR Simulator: F5 start/stop, F6/F7 previous/next song,
    /// F8 strike density, F9 partner on/off, [ and ] calibration, F10 a test strike now.
    /// </summary>
    public class RhythmPanel : MonoBehaviour
    {
        public RhythmMode mode;

        static readonly Color Slate = new Color(0.17f, 0.19f, 0.23f);
        static readonly Color GoColor = new Color(0.20f, 0.52f, 0.30f);
        static readonly Color StopColor = new Color(0.62f, 0.22f, 0.20f);
        static readonly Color BoardColor = new Color(0.30f, 0.21f, 0.14f);

        FingerButton startButton, everyButton, partnerButton;
        TextMeshPro status, hud;
        int everyChoice; // 0 = auto, 1 = every beat, 2 = every other beat
        float refreshAt;

        void Start()
        {
            var board = GameObject.CreatePrimitive(PrimitiveType.Cube);
            board.name = "Board";
            var col = board.GetComponent<Collider>();
            if (col != null) Destroy(col);
            board.transform.SetParent(transform, false);
            board.transform.localPosition = new Vector3(0f, -0.016f, 0f);
            board.transform.localScale = new Vector3(0.30f, 0.012f, 0.18f);
            board.GetComponent<Renderer>().material.color = BoardColor;

            // far row
            MakeButton(-0.105f, 0.037f, "< Song", Slate, () => ChangeSong(-1));
            MakeButton(-0.035f, 0.037f, "Song >", Slate, () => ChangeSong(+1));
            startButton = MakeButton(0.035f, 0.037f, "Start", GoColor, ToggleStart);
            everyButton = MakeButton(0.105f, 0.037f, "Beats: auto", Slate, CycleEvery);
            // near row
            MakeButton(-0.105f, -0.037f, "Cal -10 ms", Slate, () => mode.Music.Nudge(-10));
            MakeButton(-0.035f, -0.037f, "Cal +10 ms", Slate, () => mode.Music.Nudge(+10));
            MakeButton(0.035f, -0.037f, "Auto cal", Slate, AutoCalibrate);
            partnerButton = MakeButton(0.105f, -0.037f, "Partner: on", Slate, TogglePartner);

            // song info standing on the far edge, upright (the board is tilted 15 deg toward you)
            status = MakeText("Song info", transform, 0.15f, new Vector2(0.34f, 0.09f));
            status.transform.localPosition = new Vector3(0f, 0.05f, 0.1f);
            status.transform.localRotation = Quaternion.Euler(15f, 0f, 0f);

            hud = MakeText("Score", mode.transform, 0.3f, new Vector2(0.8f, 0.3f));
            hud.gameObject.SetActive(false);

            if (mode.Music.SongCount > 0) mode.Music.Select(0); // find the first song's beat now
        }

        FingerButton MakeButton(float x, float z, string text, Color color, Action action)
        {
            var go = new GameObject("Button " + text);
            go.transform.SetParent(transform, false);
            go.transform.localPosition = new Vector3(x, 0f, z);
            var b = go.AddComponent<FingerButton>();
            b.mode = mode;
            b.onPress = action;
            b.Build(text, color);
            return b;
        }

        static TextMeshPro MakeText(string name, Transform parent, float fontSize, Vector2 rect)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            var t = go.AddComponent<TextMeshPro>();
            t.fontSize = fontSize;
            t.alignment = TextAlignmentOptions.Center;
            t.textWrappingMode = TextWrappingModes.NoWrap;
            t.rectTransform.sizeDelta = rect;
            t.color = Color.white;
            return t;
        }

        void Update()
        {
            if (mode == null || mode.Music == null || mode.Game == null || hud == null) return;
            var kb = Keyboard.current;
            if (kb != null)
            {
                if (kb.f5Key.wasPressedThisFrame) ToggleStart();
                if (kb.f6Key.wasPressedThisFrame) ChangeSong(-1);
                if (kb.f7Key.wasPressedThisFrame) ChangeSong(+1);
                if (kb.f8Key.wasPressedThisFrame) CycleEvery();
                if (kb.f9Key.wasPressedThisFrame) TogglePartner();
                if (kb.leftBracketKey.wasPressedThisFrame) mode.Music.Nudge(-10);
                if (kb.rightBracketKey.wasPressedThisFrame) mode.Music.Nudge(+10);
                if (kb.f10Key.wasPressedThisFrame) mode.Game.PlayerStrike(mode.Music.SongTime(), false);
            }
            bool showHud = mode.Game.Active;
            if (hud.gameObject.activeSelf != showHud) hud.gameObject.SetActive(showHud);
            if (showHud)
            {
                hud.transform.position = mode.HudAnchor;
                RhythmGame.Billboard(hud.transform);
            }
            if (Time.unscaledTime >= refreshAt) { refreshAt = Time.unscaledTime + 0.1f; Refresh(); }
        }

        void Refresh()
        {
            var m = mode.Music;
            var g = mode.Game;
            bool running = g.Active || m.IsPlaying;
            startButton.SetLabel(m.IsLoading ? "Loading" : running ? "Stop" : "Start");
            startButton.SetColor(running ? StopColor : GoColor);
            everyButton.SetLabel(everyChoice == 0 ? "Beats: auto" : everyChoice == 1 ? "Every beat" : "Every 2nd beat");
            partnerButton.SetLabel(mode.partnerEnabled ? "Partner: on" : "Partner: off");

            string song = m.SongCount == 0 ? "No songs"
                : string.Format("{0} ({1}/{2})", m.SongIndex >= 0 ? m.SongName : m.NameOf(0), Math.Max(0, m.SongIndex) + 1, m.SongCount);
            string info = m.IsLoading || m.Analysis == null ? m.Status
                : string.Format("{0:0} BPM{1}", m.Analysis.Bpm, g.Active ? (g.Every == 1 ? ", a strike every beat" : ", a strike every 2nd beat") : "");
            string cal = string.Format("Calibration {0:+0;-0;0} ms", m.CalibrationMs);
            var err = g.MeanErrorMs;
            if (err.HasValue) cal += string.Format(", you hit {0:0} ms {1}", Math.Abs(err.Value), err.Value >= 0 ? "late" : "early");
            status.text = song + "\n" + info + "\n" + cal;

            if (!g.Active) return;
            string role = mode.PlayerPounding ? "Strike as the ring meets the rim"
                : mode.Partner.Driving ? "Turn the dough between strikes, hands out as the ring closes"
                : "Pick up the kine to play";
            hud.text = string.Format("<size=150%>{0}</size>  x{1}\nCombo {2}   best {3}\n<size=60%>{4}</size>", g.Score, g.Mult, g.Combo, g.MaxCombo, role);
        }

        void ToggleStart()
        {
            var m = mode.Music;
            var g = mode.Game;
            if (m.IsLoading) return;
            if (g.Active || m.IsPlaying) { g.StopGame(); m.Stop(); return; }
            if (m.Analysis == null) { m.Select(Math.Max(0, m.SongIndex), ok => { if (ok) StartNow(); }); return; }
            StartNow();
        }

        void StartNow()
        {
            string wait = RhythmStoryBridge.WaitReason;
            if (wait != null) { mode.Music.SetStatus(wait); return; }
            if (mode.Music.Play()) mode.Game.StartGame(everyChoice);
        }

        void ChangeSong(int delta)
        {
            var m = mode.Music;
            if (m.SongCount == 0 || m.IsLoading) return;
            bool wasRunning = mode.Game.Active;
            mode.Game.StopGame();
            int target = m.SongIndex < 0 ? (delta > 0 ? 0 : m.SongCount - 1) : m.SongIndex + delta;
            m.Select(target, ok => { if (ok && wasRunning) StartNow(); });
        }

        void CycleEvery()
        {
            everyChoice = (everyChoice + 1) % 3;
            if (mode.Game.Active) mode.Game.StartGame(everyChoice);
        }

        void TogglePartner() => mode.partnerEnabled = !mode.partnerEnabled;

        void AutoCalibrate()
        {
            var e = mode.Game.MeanErrorMs;
            if (!e.HasValue) return;
            mode.Music.Nudge(-(int)Math.Round(e.Value));
            mode.Game.ClearTimingHistory();
        }
    }
}
