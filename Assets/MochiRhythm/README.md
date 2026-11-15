# MochiRhythm: rhythm mode

Pound and knead to the beat of a song. This is a port of the rhythm mode from the three.js version (`immerse/mochi-pounding`, `src/rhythm/`). It keeps that version's beat tracker, timing windows and scoring, and runs on this branch's own kine, usu and dough (`MochiStation.Sim`), so it works with either solver.

## How it plays

Rhythm mode adds itself at runtime to any scene with a hand-tracked `MochiStation` (`MochiStandalone`, `MochiPounding`), so no scene file changes. It does nothing until someone presses **Start**.

- **Panel.** A small board lies on the table to the kneader's left and moves with the station. Press its buttons with a fingertip or the tip of a Touch controller:
  - `< Song` and `Song >`
  - `Start` / `Stop`
  - `Beats`: auto, every beat, or every 2nd beat
  - `Cal -10 ms` and `Cal +10 ms`, and `Auto cal`, which applies your average timing error
  - `Partner` on/off
- **Rings.** For each note a ring closes onto the usu's rim and meets it on the beat. The rim pulses on every beat. The score floats beyond the usu while a game runs.
- **Pounder.** Pick up the kine (hand or controller, through `MochiGrabber`), or use `KineDriver` Controller mode. Strikes are judged by when you hear them:
  - within 60 ms: Perfect
  - within 130 ms: Good
  - 250 ms late: a miss
  - combo multiplier up to x4
  - striking while your free hand is under the kine: "Ouch! Hands!"
- **Kneader.** While nobody holds the kine and Partner is on, the kine pounds every note by itself. It uses `KineDriver`'s auto-pound motion (same raised height, impact speed, sink depth, dwell and lift speed), but each fall is timed to land on the note. It tips the handle over the rim near the bottom of the stroke, the same as the team's auto-pound (`KineDriver.AutoPoundRotation`). Turn the dough between strikes ("Turned" scores) and get your hands out before it lands ("Hands!" breaks the combo). Grab the kine to take over; stop the game and it goes back to its rest spot.
- **Sound.** `MochiSound` routes dough strikes, usu/table knocks, dough slaps and dough release through the bundled Satie scene, and publishes kneading speed and the free-play/rhythm state. Impact takes are cut to single hits. If Satie is unavailable, strikes and knocks fall back to the original mallet clips (pitched up for knocks). See `Satie/README.md`.

**Keyboard** (Editor / Meta XR Simulator):

| Key | Action |
|---|---|
| F5 | start/stop |
| F6 / F7 | previous / next song |
| F8 | beat density |
| F9 | partner on/off |
| `[` and `]` | calibration |
| F10 | test strike |

## Songs

Put mp3, wav, ogg or aiff files in `Resources/MochiRhythm/Music/`. They are imported Decompress On Load and are not preloaded (`Editor/MochiRhythmAudioImport.cs`), so on the headset only the chosen song is decoded, the first time it is picked. Finding its beat takes about a second.

On the headset you can also push songs to `/sdcard/Android/data/com.immersethebay.mochipound/files/Music/`; they are picked up at start without a rebuild.

`*.mp3` files in the Music folder are git-ignored, because they are copyrighted songs.

## Calibration

Audio reaches your ears later than Unity's audio clock says. Hit a few notes, press `Auto cal`, and repeat until the panel says you are within about 10 ms. The value is kept in PlayerPrefs.

## Files

- `Scripts/BeatAnalyzer.cs`: tempo and beat tracking, line for line from `beats.js`. It uses windowed band-energy onsets, autocorrelation tempo with a ~110 BPM prior, and Ellis dynamic-programming beats. On five synthetic songs (72-150 BPM) its results are identical to the JavaScript version's, with a median beat error of 1.8-2.9 ms.
- `Scripts/RhythmMusic.cs`: loads songs, beat-tracks them on a worker thread, and loops playback with `PlayScheduled`. `SongTime()` is the song time heard now (the DSP clock minus the output latency, smoothed, plus calibration). Pauses on app pause and the system menu.
- `Scripts/RhythmGame.cs`: notes, rings, rim pulse, judging, score, combo and popups (from `rhythm.js`).
- `Scripts/RhythmMode.cs`: finds the station, kine, grab zone, usu and grabber. Detects strikes (the `KineTool` face reaching the dough surface while moving down), plays the sounds, tracks the kneader's hands, and builds the other parts.
- `Scripts/MochiSound.cs`: binds Satie's hosts to the station, routes strikes and knocks, detects hand slaps and estimates the native solver's dough release, and publishes kneading speed and game state.
- `Scripts/SatieStartup.cs`: starts Satie in its `StereoMix` output when the spatializer named in Project Settings > Audio ("Meta XR Audio") is not actually loaded. The Meta XR Audio SDK is not installed, and Satie's default output would otherwise set `spatialize` on its sources, which plays silence on Quest. With a working spatializer it leaves Satie's own start alone. Define `MOCHI_SATIE_DEFAULT_START` to turn it off.
- `Scripts/RhythmPartner.cs`: the on-beat auto-pound.
- `Scripts/DoughHeight.cs`: the dough's surface under the kine, from `IMochiSim.RequestParticleReadback`. It is asynchronous: 8 reads/s while playing, 2 when idle.
- `Scripts/HandPoints.cs`: palms and fingertips through `HandBoneCompat`.
- `Scripts/FingerButton.cs`, `Scripts/RhythmPanel.cs`: the panel.
- `Scripts/RhythmModeBootstrap.cs`: attaches rhythm mode at scene load. Define `MOCHI_RHYTHM_OFF` to stop that; `RhythmModeBootstrap.Attach()` still adds it on demand.

Rhythm mode pauses itself (and goes quiet) while `MochiStrikeRecorder` or `MochiTestBench` runs, so it stays out of their frames.

## Verification

The rhythm, sound and collision scripts passed the local C# type check on 2026-11-14 with zero warnings and errors. That check uses Unity/Satie API stand-ins plus the branch's gameplay sources; it does not compile or run the Unity project. Unity compilation, play-mode recordings (including `tiltOverRim` on and off), sound playback and Quest testing are still pending.
