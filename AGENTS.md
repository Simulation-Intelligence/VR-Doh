# Agent guide: MochiPound

Instructions for AI coding agents (and people) working in this repository. `CLAUDE.md` imports this file, so keep everything here.

## What this is

A mixed-reality mochi-pounding game for Meta Quest 3. One player kneads virtual mochi with tracked hands; the other swings a virtual kine (mallet) into an usu (mortar). It is a fork of [VR-Doh](https://github.com/Simulation-Intelligence/VR-Doh), which supplies the hand tools, marching-cubes renderer and the original Taichi MPM solver.

- Unity **6000.6.9f1**, **Built-in render pipeline** (URP is installed as a package but not used; URP materials render magenta).
- Meta XR Core SDK and Interaction SDK 209, OpenXR.
- Working branch: `unity6-upgrade`. Fork remote: `https://github.com/Znasif/MochiPound`. The original VR-Doh repository is the other remote; never push there.

**The goal is how it looks and feels, not physical accuracy.** A convincing strike and convincing kneading at headset frame rate beat a correct constitutive model. Judge changes by images and interaction, and justify shortcuts by that.

## Two dough solvers

| | Native solver (current direction) | Taichi solver (legacy) |
|---|---|---|
| Runs on | Quest 3 standalone, and the Editor | PC only, over Quest Link |
| Code | `Assets/Resources/MochiSolver.compute`, `Assets/Scripts/Mochi/MochiSolver.cs` | `scripts/*.py` compiled to `Assets/Resources/TaichiModules/*.tcm`, hosted by `Assets/Scripts/Mpm3DMarching.cs` |
| Scene | `Assets/Scenes/MochiStandalone.unity` | `Assets/Scenes/MochiPounding.unity` |
| Size | about 5,800 particles, 32x26x32 grid, 11 mm cells | about 23,000 particles, 48^3 or 64^3 grid |

New work goes into the native solver. Do not add Python or Taichi dependencies to the standalone path. The Taichi path stays working for comparison; if you touch it, the Taichi version must be 1.7.3 to match `Assets/Plugins/X86_64/taichi_c_api.dll`.

Both solvers implement `IMochiSim` (`Assets/Scripts/Mochi/IMochiSim.cs`). Gameplay code talks to `MochiStation.Sim`, never to a concrete solver.

## Layout

- `Assets/Scripts/Mochi/`
  - `MochiStation.cs`: places the usu and the solver, hands the tool list to the solver. If its `solver` field is set it uses the native solver, otherwise `mochi` (Taichi).
  - `MochiSolver.cs`: native solver host. Buffers, substep loop, tool upload, fist detection, marching-cubes mesh. All tuning fields live here.
  - `KineTool.cs`, `UsuTool.cs`, `PalmTool.cs`, `PrimitiveTool.cs`: colliders built from medial primitives (two spheres = cone/capsule, three spheres = slab).
  - `KineDriver.cs`: moves the kine (held controller, auto-pound, or external). Auto-pound tips the handle up near the bottom of the stroke (`tiltOverRim`, `RimTilt`, `AutoPoundRotation`). The tilt is just enough for the handle to clear the usu rim: about 18° at `bottomHeight`, and level once the face is more than about 7 cm above the floor. Without it the handle, level with the middle of the head, passed through the rim.
  - `KineContact.cs` + `KineCollision.cs`: kine-versus-solid collisions, which the solver doesn't do (it only collides dough with tools). After the kine is posed each frame, it is kept out of three things:
    - the usu
    - the table top
    - free tracked hands, while the kine moves; the hand holding it is ignored

    The handle hits the rim and the table, but not hands by default (it reaches over the kneader's wrists). `KineCollision` is a port of the three.js scene's `src/kineContact.js` (rewritten 2026-11-15 because the kine jumped between surfaces):
    - **The shapes that collide are the shapes that are drawn.** The usu skin `MR_UsuSkin` (or `UsuTool`'s lathe without it) and the kine skin `MR_KineSkin` (head with its bands, tapered handle, grip wrap, knob; or the `KineTool` cylinder without it) are each the exact solid of revolution their mesh is lathed from. The profiles copy `UsuKineSkinBuilder`'s numbers; at start `KineContact` logs how far the drawn vertices are from them (`[kine] collision shapes ...`, a warning above 3 mm). Change both together.
    - **Contacts:** the deepest point of the kine's surface inside the usu or the table, the usu's convex corners (the rim's edges, the band) inside the kine, and the hand balls.
    - **Resolution:** all contacts are cleared by one pivot about the holding hand (or the grip's middle when nobody holds it), or a push when the push runs along the handle. It is bisected to the smallest move, so the kine rests touching. A move may be at most about the overlap's size; if no local move clears it, the kine is wedged and keeps its last pose, instead of swinging over or around what it hit.
    - **Motion:** marched from last frame's resolved pose by the free space around the kine (at least the handle's radius near something). Once a step is blocked and keeps less than half its advance, the kine stays on the near side. A held kine rests on the surface while the real hand carries on through it, and comes back to the hand when it moves away. A target that jumps more than 0.3 m in a frame (put down, `KineDriver.Rest`) is placed, not marched.
    - Impacts raise `Contact` and buzz the holding controller.
    - It is added at runtime to every `KineTool` (`KineContactBootstrap`; define `MOCHI_KINE_CONTACT_OFF` to stop that).
    - It is idle while the recorder or test bench runs.
  - `MochiGrabber.cs`, `GrabZone.cs`, `KineSafety.cs`: picking up the kine and the usu rim, and the swing-near-hands warning. The rim zone is not in the grabber's list in `MochiStandalone`; placement is automatic.
  - `UsuTablePlacer.cs`: puts the station on the nearest scanned table from the MR Utility Kit room (the `MRUK` object in `MochiStandalone`, World Lock off because the rig is moved for the Moon scene), at the table's near edge, and cuts the table out of the VR world so the real table shows in passthrough. With no table it opens Space Setup once (on device only). In the Editor the Meta XR Simulator's synthetic room supplies the table; `Office00` is the prefab fallback.
  - `MochiStrikeRecorder.cs`: development aid, see Testing.
  - `MochiTestBench.cs`: older scripted bench with a JSON report; written for the Taichi solver, works through `IMochiSim`.
- `Assets/Resources/MochiSolver.compute`: the whole native simulation. Read its header comment first.
- `Assets/Scripts/MpmTools/MatHand.cs`: tracked hand as about 95 medial primitives per hand.
- `Assets/MarchingCubes/`: isosurface mesh builder used by both solvers.
- `Assets/Shaders/Mochi/MochiSurface.shader`, `Assets/Materials/Mochi/`: dough look.
- `Assets/MochiRhythm/`: rhythm mode (pound and knead to a song), see "Rhythm mode" below and its README.

## How the native solver works

MLS-MPM in SI units, in the solver transform's local space (metres). Two dispatches per substep:

1. `Nodes`: particle-to-grid as a gather over per-cell particle lists (no float atomics, which mobile GPUs handle badly), then gravity, tool contact, tack, floor and walls.
2. `Particles`: grid-to-particle, advection, elastic strain update with a viscoplastic return, and binning for the next substep.

Once per frame: `ToolSdf` (signed distance, normal, velocity and tack of the nearest tool at every node; the usu wall is analytic) and `Density` (node mass resampled to the render grid for marching cubes).

Deliberately non-physical parts, which carry the illusion:

- **Tack.** Dough pressed by a tool with tack > 0 becomes bonded, and the bond spreads through touching dough up to `pullReach`. While the tool pulls away, bonded dough is given `tool velocity x falloff(distance)`, which stretches the lump into a tent rather than tearing a skin off. Bonds age only while the tool pulls, so they all release together after the tool's tack time. Relying on real adhesion stress here was tried and fails: the bonded skin tears off as crumbs.
- **Grip.** A closed fist has a very long tack and carries the dough within `gripReach` rigidly.
- **Per-tool tack** is chosen by tool type in `MochiSolver.TackFor`: kine 0.15 s (dry), open hand 0 (wet, slides off), fist 30 s.

Tool velocities are displacement divided by *simulated* frame time, so when the substep cap puts the dough in slow motion a tool still sweeps through it consistently. Do not change that back to `Time.deltaTime`.

## Target behaviour

Targets, with current native-solver status as of 2026-11-14:

| Target | Status |
|---|---|
| Kine sinks to about half the lump height on a 4 m/s strike | Sink depth is set by the driver; looks right. Since 2026-11-14 the auto-pound kine tips up to about 18° in the bottom ~5 cm of the stroke (`tiltOverRim`). The face then meets the dough slightly tilted and the head untilts during the first few cm of the lift, while the tent forms. **Not yet re-recorded**: compare `strike` recordings with `tiltOverRim` on and off before trusting the tent targets below |
| On lift the dough follows the kine into a tent about as tall as the lump, then lets go after about 0.15 s | Working: a connected column up to about 2.3x lump height, released with a pointed peak, repeatable strike after strike. Needs `pullShape` 1 (an uneven stretch tears a cap off under the kine) and the shoulder plate in `KineTool` (dough caught in the groove behind the face rode up with the kine) |
| After release the tent falls back to a round mound in 0.3-0.4 s | Drops within about 0.3 s, then takes about 1.5 s to finish rounding |
| Dents round off instead of staying as permanent prints | Working in the Editor. The dough is soft (`yieldStress` 80 Pa, `relaxTime` 0.05 s) and pulled toward the usu axis (`centering`), so it flows back into a dome: a hand cut that splits the lump refills in a couple of seconds, and repeated strikes start from the same mound. A cohesion (fake surface tension) term was tried first and collapsed the lump; do not bring it back |
| Open (wet) hands slide off; a fist grips and lifts | Hands push only. **Grip is switched off** (`fistTack` 0 in the scene): with real hands a half-closed kneading hand kept triggering it and it ripped the lump off the floor in pieces |
| Dough stays in one piece under real fingers | Working in the Editor with a rigid copy of the tracked-hand collider (recorder `handShape`): stray bits squeezed out between fingers are braked (`strayDensity`, `strayDrag`) and dough speed is capped (`maxDoughSpeed`). Not yet confirmed on the headset |
| Real hands stay visible in passthrough | `MochiHandCutout` draws the hand collider spheres with `Mochi/PassthroughCutout`, which leaves a hole for passthrough. Verified in the Editor only |
| The lump keeps its volume through repeated kneading | Working since `crowdStiffness` was added. Before it, particles packed into fewer cells each cycle (the tracked volume ratio J still read 1), overflowed the per-cell lists and vanished from the picture. After repeated grips the lump is still lumpy and can shed specks |
| Real tracked hands (`MatHand`) knead correctly | **Untested.** Only the scripted `PalmTool` has been run |
| Real time at 72 Hz on Quest 3 | Holds 72 fps with the lump at rest and nobody touching it: app GPU time about 9.4 ms, GPU about 91% busy (2026-11-14, first run). Not yet measured during strikes or kneading, where the 12-substep cap will put the dough in slow motion. There is almost no headroom |
| The kine can't pass through the usu, the table or a free hand | `KineContact`/`KineCollision` (rewritten 2026-11-15 as a port of the three.js collision, against the drawn skins) plus `KineDriver.tiltOverRim`. An off-Editor test of `KineCollision` with the skin shapes passes, with overlap and resting gap 0.000 mm in every case: head into the bowl floor, into the bowl wall and onto a hand; handle across the rim; a 12 m/s swing into the wall, which stops touching it; a swing over the rim; a held kine pushed 2 cm a frame down through the rim, which rests on it; and one swept 3 cm a frame through the body, which stays outside with the handle level and rides over with it raised. No frame moves the head more than the hand moved, and the kine returns exactly to the hand. Cost: 0.2 ms a frame in contact and 0.04 ms in the air on one juno2 core. **Not yet run in Unity or on the headset.** The handle still passes over hands (off by default) |

Update this table when you change any of it.

## Tooling

Use the Unity CLI for all Editor operations and the `metavr` CLI for all device and Meta-specific work. Read their built-in guides instead of guessing:

```bash
unity --no-banner --no-pager skill show          # Unity CLI guide
unity --no-banner command --project-path <proj>  # commands exposed by the open Editor
npx -y metavr@latest skills show metavr-cli      # metavr reference (use `metavr` directly if on PATH)
npx -y metavr@latest docs search "<topic>"       # live Meta developer docs; query before answering Meta XR questions
```

Rules that have already cost time:

- Wait for `unity status --until-ready` after every recompile, domain reload or play-mode change.
- `unity command script eval_file` code cannot contain `using` lines; fully qualify type names.
- One Editor per project folder. If someone else has this folder open, use a git worktree and open that.
- Test in play mode with the Meta XR Simulator active (menu `Window/Meta/Meta XR Simulator/Activate`) and deactivate it when you finish, so the next person can use Link. Its first frame can take over a minute.
- Use `metavr device ...`, `metavr app ...`, `metavr log`, not raw `adb`.
- Save scenes through the Editor (`unity command scene save` or an eval), never by editing `.unity` files as text.

## Required verification

After any change to code, shaders or scenes:

1. Recompile and confirm the console has no errors (`unity command console get --level error`).
2. Enter play mode in `MochiStandalone` and run the recorder (below) for the behaviour you touched.
3. Look at the saved frames yourself before reporting. Numbers alone have been misleading on this project: an earlier metric scored torn-off crumbs as a tall cone.
4. Capture Game and Scene view screenshots (`unity command capture screenshot --view game|scene --output <abs path>.png`).
5. Report failures, console errors and visuals that look wrong as they are.

Put verification output under `Temp/`. Unity may clear `Temp/` when the Editor closes.

## Testing the dough

`MochiStrikeRecorder` is added at runtime, not saved in the scene. In play mode, evaluate:

```csharp
var g = new UnityEngine.GameObject("Rec"); g.SetActive(false);
var r = g.AddComponent<MochiStrikeRecorder>();
r.tag = "my_run";          // output folder name
r.mode = "strike";         // or "knead"
g.SetActive(true);
```

- `strike`: the kine auto-pounds; one strike is recorded from the moment the kine starts to fall.
- `knead`: a scripted palm does an open-hand press, a sideways push, then a grip-and-lift.
- Output goes to `Temp/mochi_native/<tag>/`: side-view frames with the usu wall hidden, `series.txt` (per-frame kine height, bonded particle count, highest point, mean volume ratio), and `summary.txt` (fps, substeps per frame, simulated/real time ratio).

Solver fields on `MochiSolver` can be changed live in play mode before starting a recording, which is the fastest way to tune.

**Values saved in the scene override the defaults in `MochiSolver.cs`.** Changing a default in code does nothing for a field the scene already stores; set the value on the component in `MochiStandalone.unity` and save the scene. This silently kept grip on and damping low on the headset for several builds.

For anything involving fingers, test with the real hand collider, not the flat palm: `r.handShape = true; r.jitter = 0.0015f;` uses a rigid copy of the 95-primitive tracked hand. Thin fingers behave very differently from the palm (they squeeze dough out between them).

On the headset the solver logs a `MochiStats` line per second (`metavr log -t Unity`): frame rate, substeps, fastest particle, how many are above the rim, whether each hand counts as a fist.

## Rhythm mode

`Assets/MochiRhythm/` (details in its README) adds a song-driven game on top of the station. It is a port of the three.js prototype's rhythm mode, with the same beat tracker, timing windows and scoring.

- **Attach.** `RhythmModeBootstrap` adds a `MochiRhythm` object at scene load to any scene whose `MochiStation` uses tracked hands. No scene file is changed. Define `MOCHI_RHYTHM_OFF` to disable this.
- **Idle.** Until someone presses Start on its panel (on the table, left of the usu), it only plays sounds. `MochiSound` routes strikes, usu/table knocks, hand slaps and native-solver dough release through Satie, and publishes kneading speed and free-play/rhythm state. Strikes and knocks fall back to the original mallet clips if Satie is unavailable. `SatieStartup` starts Satie in `StereoMix` output when the spatializer named in Project Settings > Audio is not loaded: "Meta XR Audio" is named but its SDK is not installed, and Satie's default per-voice sources would then be spatialized into silence on Quest. The scene and cut-take sources are in `Satie/`, the compiled scene is in `Assets/Resources/SatieCore/`, and the embedded runtime is in `Packages/world.satie.unity/`.
- **Recorder and test bench.** It pauses itself while `MochiStrikeRecorder` or `MochiTestBench` runs, so recordings are unchanged.
- **Gameplay code** goes only through `MochiStation.Sim` (particle readback for the dough surface), `KineTool` (face = local origin), `KineDriver`, `GrabZone.Held` and `UsuTool` dimensions. It changes none of their code or fields.
- **Partner.** While a game runs and nobody holds the kine, `RhythmPartner` moves the kine itself (`KineDriver` must be in External mode). It uses the AutoPound speeds and heights, with each fall timed to land on a note. It returns the kine with `KineDriver.Rest()` when the game stops.
- **Cost.** One asynchronous particle readback 8 times a second while playing (2 when idle) and a few line renderers. This has not been measured on the headset.
- **Songs** go in `Assets/MochiRhythm/Resources/MochiRhythm/Music/` (mp3s there are git-ignored), or on the headset in `/sdcard/Android/data/com.immersethebay.mochipound/files/Music/`.
- **Editor keys:**
  - F5: start/stop
  - F6 / F7: previous / next song
  - F8: beat density
  - F9: partner on/off
  - `[` and `]`: calibration
  - F10: test strike

Status (2026-11-14): written and type-checked off-Editor (against stand-ins plus this branch's `GrabZone`, `KineDriver`, `KineTool`, `PrimitiveTool`, `MochiGrabber`, `HandBoneCompat` and `IMochiSim` sources). **Not yet compiled in Unity or run in play mode or on the headset.**

## Building for Quest (standalone)

```bash
unity command build run --target Android --outputPath Builds/MochiStandalone.apk \
  --scenes '["Assets/Scenes/MochiStandalone.unity"]' --confirm true --project-path <proj>
unity command build status --project-path <proj>     # poll until completed
metavr device list
metavr app install Builds/MochiStandalone.apk
```

- The first Android build switches the platform and re-imports every asset. Expect well over 40 minutes, with the Editor unreachable meanwhile. Later builds reuse the cache.
- If a dialog loops on "SDK Platform Tools version 0.0 < 37.0.1", Unity is pointed at an external Android SDK. In Preferences > External Tools, tick "Android SDK Tools Installed with Unity". That is a per-user Unity preference, not a project setting.
- The Taichi `.so` files under `Assets/Plugins/Android` ship in the APK but are not used by `MochiStandalone`.
- The standalone build installs and runs on Quest 3 with passthrough (first run 2026-11-14). The performance knobs are `cellSize`, `gridCells`, `particlesPerCell`, `maxSubsteps`, `waveSpeed` and `renderScale` on `MochiSolver`. Frame statistics are in `metavr log` on the `VrApi` lines (`FPS=`, `App=` is app GPU time).
- `Assets/Plugins/Android/AndroidManifest.xml` is generated from the Meta project config. If hand tracking or passthrough entries are missing, or the build fails on a `com.oculus.supportedDevices` manifest merge conflict, regenerate it with `OVRManifestPreprocessor.GenerateOrUpdateAndroidManifest(true)` (or Window > Meta > Tools > Android Manifest Tool).
- `metavr app install` needs a path the calling shell can read (a WSL path from WSL, not `D:\...`). The package id is `com.immersethebay.mochipound` and the app shows as "MochiPound" on the headset.

## Boundaries

- Preserve existing scenes and assets. `MochiPounding.unity` and the original VR-Doh scenes are not to be changed for standalone work; change `MochiStandalone.unity`.
- Do not commit `ProjectSettings/`, `Packages/` or `Assets/XR/` changes that the Editor made on its own (platform switches, simulator activation). Check `git status` and leave them out unless the change is the point of the commit.
- Do not commit `Library/`, `Temp/`, `Logs/`, `Builds/` or `UserSettings/`.
- Keep the `IMochiSim` members and `MochiStation.FloorY`, `SimToWorld`, `SimToStation` stable; gameplay and tests depend on them.
- No pushes to the VR-Doh `origin` remote, no force pushes, no history rewrites.
- Use dry-run and confirm parameters on destructive Editor commands.
