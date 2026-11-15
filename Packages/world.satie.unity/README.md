# Satie for Unity

Plays a Satie scene in Unity through Satie's portable core. The core decides what
sounds, when and how loud, rings the rooms and muffles what is behind a wall; Unity's
panning, or the spatializer plugin the project has selected, gives each sound its
direction.

This folder is written by the `satie-unity` command (the line `satie_export` returns).
Do not edit it: the command replaces it when a newer version exists.

## What is here

- `Runtime/`: `SatieCoreBehaviour` (the component) and `SatieCore` (the calls a game makes).
- `Editor/`: the mixer, under Window > Satie > Mixer.
- `Plugins/`: the native library for Windows x64, macOS, Linux x64 and Android arm64.
  Other build targets (iOS, WebGL, consoles) have no library yet and stay silent.

## How it starts

Nothing is assigned in the editor. When `Assets/Resources/SatieCore` holds a compiled
scene, a `Satie` object with the component starts after the first scene loads and
stays across scene changes. Add `SatieCoreBehaviour` to an object of your own only to
change its settings (the listener, the master gain, who gives direction).

## Telling the sound what happens

```csharp
using Satie.Core;

SatieCore.Fire("door_open", door.position);       // an event at a place
SatieCore.HostEvent("crate", "drop");             // an event from a bound object
SatieCore.SetHostSurface("player", "gravel");     // before the step that reads it
SatieCore.Signal("engine_load", 0.7f);            // a continuous control
SatieCore.State("combat");                        // the game changed mode
SatieCore.Bind("door", door.transform);           // when the object's name differs
```

A sound written `at <id>` follows the object with that name. An object nobody finds
plays at the position the scene gave, and the console says so once.

## Changing the sound

Mix in Window > Satie > Mixer while the game plays, press Save, then run the
`satie-unity` command again (or keep it running with `--watch`): the saved changes go
into `Satie/scene.satie`, the scene is compiled again and a playing editor reloads it.

Sound by Satie (https://www.satie.world): please keep this line in the game's credits.
