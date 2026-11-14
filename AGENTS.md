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
  - `KineDriver.cs`: moves the kine (held controller, auto-pound, or external).
  - `MochiGrabber.cs`, `GrabZone.cs`, `KineSafety.cs`: picking up the kine and the usu rim, and the swing-near-hands warning.
  - `MochiStrikeRecorder.cs`: development aid, see Testing.
  - `MochiTestBench.cs`: older scripted bench with a JSON report; written for the Taichi solver, works through `IMochiSim`.
- `Assets/Resources/MochiSolver.compute`: the whole native simulation. Read its header comment first.
- `Assets/Scripts/MpmTools/MatHand.cs`: tracked hand as about 95 medial primitives per hand.
- `Assets/MarchingCubes/`: isosurface mesh builder used by both solvers.
- `Assets/Shaders/Mochi/MochiSurface.shader`, `Assets/Materials/Mochi/`: dough look.

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
| Kine sinks to about half the lump height on a 4 m/s strike | Sink depth is set by the driver; looks right |
| On lift the dough follows the kine into a tent about as tall as the lump, then lets go after about 0.15 s | Working, tent about 1.6x lump height, clean release, no crumbs |
| After release the tent falls back to a round mound in 0.3-0.4 s | Falls back quickly; a slight wobble remains |
| Dents round off instead of staying as permanent prints | **Not working.** Palm prints and lumps stay. A cohesion (fake surface tension) term was tried and collapsed the lump; it was removed |
| Open (wet) hands slide off; a fist grips and lifts | Hands push only. **Grip is switched off** (`fistTack` 0 in the scene): with real hands a half-closed kneading hand kept triggering it and it ripped the lump off the floor in pieces |
| Dough stays in one piece under real fingers | Working in the Editor with a rigid copy of the tracked-hand collider (recorder `handShape`): stray bits squeezed out between fingers are braked (`strayDensity`, `strayDrag`) and dough speed is capped (`maxDoughSpeed`). Not yet confirmed on the headset |
| Real hands stay visible in passthrough | `MochiHandCutout` draws the hand collider spheres with `Mochi/PassthroughCutout`, which leaves a hole for passthrough. Verified in the Editor only |
| The lump keeps its volume through repeated kneading | Working since `crowdStiffness` was added. Before it, particles packed into fewer cells each cycle (the tracked volume ratio J still read 1), overflowed the per-cell lists and vanished from the picture. After repeated grips the lump is still lumpy and can shed specks |
| Real tracked hands (`MatHand`) knead correctly | **Untested.** Only the scripted `PalmTool` has been run |
| Real time at 72 Hz on Quest 3 | Holds 72 fps with the lump at rest and nobody touching it: app GPU time about 9.4 ms, GPU about 91% busy (2026-11-14, first run). Not yet measured during strikes or kneading, where the 12-substep cap will put the dough in slow motion. There is almost no headroom |

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
