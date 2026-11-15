# satie-scene: Satie scene for Unity

8 sounds. Sound by Satie (https://www.satie.world): please keep this line in the game's credits.

## What is in the project

- `Satie/`: the scene as you keep it (`scene.satie`, `samples.json`, `scene.contract.json`). Commit it.
- `Packages/world.satie.unity`: the Satie package (the component, the mixer window, the native libraries). Commit it.
- `Assets/Resources/SatieCore`: the compiled scene the component loads. Commit it, or compile again after a checkout.
- `.satie/`: the take cache and the mixer's log. Not committed.

Nothing is assigned in the editor: press Play and a `Satie` object starts by itself. Satie decides what sounds, when and how loud, rings the rooms and muffles what is behind a wall; Unity's panning, or the spatializer your headset SDK ships (Project Settings, Audio, Spatializer Plugin), gives each sound its direction.

Build targets with a native library: Windows x64, macOS, Linux x64 and Android arm64 (Quest and phones). On any other target (iOS, WebGL, consoles) the scene is silent for now.

## Binding

A sound written `at <host>` follows the object with that name (`GameObject.Find`). When the name differs, bind it yourself: `SatieCore.Bind("door", door.transform)`. A host that is never found plays at the position the manifest gave, and the console says so once. Positions are your own world coordinates, unchanged.

## Telling the sound what happens

Add `using Satie.Core;` to the script that makes these calls.

| What happened | Call |
| --- | --- |
| An event at a place | `SatieCore.Fire("door_open", position)` |
| An event from a host | `SatieCore.HostEvent("crate", "drop")` |
| The ground under a host changed | `SatieCore.SetHostSurface("player", "gravel")` (before the event) |
| A continuous control moved | `SatieCore.Signal("engine_load", 0.7f)` |
| The game changed mode | `SatieCore.State("combat")`, `SatieCore.State(null)` |

Events this scene answers: `pound`, `rim_knock`, `tent_release`, `dough_slap`.
Signals it follows: `strike_speed` 0 to 8, `kine_speed` 0 to 8, `knead_speed` 0 to 1.5. A sound that follows a signal stays at its resting value until the game publishes it.
States: `free_play`, `rhythm`.

`Assets/MochiRhythm/Scripts/MochiSound.cs` supplies these events, binds the hosts to the station, and publishes the signals and state. `kine_speed` is disabled by default because its generated take contains separate whooshes. Dough release timing is estimated from the native solver's tack and pull-speed settings.

The generated takes contained multiple impacts. `take-edits.json` records the cuts; the edited MP3s are committed in `takes/`. Run `node Satie/use-cut-takes.mjs` before recompiling the scene with the Satie installer so the compiled bundle uses those cuts.

The game integration passed an off-Editor C# type check on 2026-11-14. Unity compilation, playback and Quest checks are pending.

## Mixing, and changing the sound

Window > Satie > Mixer lists every sound while the game plays: its level, volume, mute and solo, the signals and the states. Save writes your decisions to `.satie/authoring/edits.jsonl`. Then run the install line again without `--from` (or keep it running with `--watch`): the decisions go into `Satie/scene.satie`, the scene is compiled again and a playing editor reloads it. To change what the scene is, compose again in Satie and export again.

## What Unity does not get

- Event voices overlap: a new fire does not cut the previous one.
- Build targets other than Windows x64, macOS, Linux x64 and Android arm64 (Quest and phones): the package has no native library for them yet, so the scene is silent there.
