# Rawan Four Motion Plugin

This is the makeup/finalized N8RO Character Animation plugin based on the NathanHuman sample animation plugin structure.

It registers four custom animation codes on `animationModelNathanHuman`:

1. `Rawan Idle`
2. `Rawan Walk`
3. `Rawan Push`
4. `Rawan Climb`

The plugin follows the sample approach used by:

- `dev\samples\sim\sim-char-anim-nathan-idle-breathing`
- `dev\samples\sim\sim-char-anim-nathan-idle-alert`

Each animation produces joint override output for the 10 required major joints:

```text
leftAnkle
rightAnkle
leftKnee
rightKnee
leftHip
rightHip
leftShoulder
rightShoulder
leftElbow
rightElbow
```

## Important cleanup before testing

To avoid the old bridge plugin fighting this new plugin, temporarily disable the previous bridge DLL if it exists:

```text
D:\N8RO_2\userPlugins\sim\rawan-character-bridge.dll
```

Move it to something like:

```text
D:\N8RO_2\userPlugins\sim\_disabled\rawan-character-bridge.dll
```

The old `userPlugins\sim\rawan\character_plugin_200201852.dll` can stay there, because it will not run without the bridge.

## Install

Copy this whole folder to:

```text
D:\N8RO_2\dev\samples\sim\rawan-four-motion-plugin
```

Also copy the included mission file:

```text
human_animation_loop.lua
```

to:

```text
D:\N8RO_2\data\resources\missions\human_animation_loop.lua
```

## Build workflow

1. Close N8RO completely.
2. Open a Command Prompt or PowerShell in:

```text
D:\N8RO_2\dev\samples\sim\rawan-four-motion-plugin
```

3. Run:

```bat
open-solution.cmd
```

4. In Visual Studio, choose:

```text
Release | x64
```

5. Build the project.

Expected output:

```text
D:\N8RO_2\dev\samples\sim\rawan-four-motion-plugin\bin\release\rawan-four-motion-plugin.dll
```

Expected auto-deploy path:

```text
D:\N8RO_2\userPlugins\sim\rawan-four-motion-plugin.dll
```

## Scenario Editor setup

Open the `GenericCivilianPresence` scenario.

For the human entity, go to the `Animation` tab and make sure `SupportedAnimationList` contains these four entries:

```text
Rawan Idle
Rawan Walk
Rawan Push
Rawan Climb
```

For each entry:

```text
animationCode = same as the name above
displayName = same as the name above
loopDefault = true
speedScaleDefault = 1.0
blendSecondsDefault = 0.15 or 0.20
```

Then press `Apply` and save if available.

## Mission script

The mission script cycles all four states every 3 seconds:

```lua
local animationSequence = {
    "Rawan Idle",
    "Rawan Walk",
    "Rawan Push",
    "Rawan Climb"
}
```

Run the simulation and open the GLB viewer. The character should switch between the four custom registered animation states.

## Recording checklist

Show these in the make-up recording:

1. Visual Studio build succeeded.
2. `rawan-four-motion-plugin.dll` exists under `userPlugins\sim`.
3. Scenario Editor Animation tab contains the four Rawan animation codes.
4. `human_animation_loop.lua` contains the four Rawan codes.
5. Simulation running in N8RO.
6. GLB viewer showing the character cycling through the four motions.

