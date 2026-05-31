# Rawan Character Animation Plugin v0.1

This repository contains a closed-library character controller DLL and an N8RO simulation/visual bridge for the Character Animation term project.

## Implemented motion states

- **WALK-LIKE**: locomotion-style controller state using motion clip ID `12`.
- **PUSH-LIKE**: two-hand push-style controller state using motion clip ID `47`.
- **CLIMB-LIKE**: low-step climb-style controller state using motion clip ID `83`.

## Controlled 10 joints

The controller outputs joint overrides for these 10 joints:

- `leftAnkle`, `rightAnkle`
- `leftKnee`, `rightKnee`
- `leftHip`, `rightHip`
- `leftShoulder`, `rightShoulder`
- `leftElbow`, `rightElbow`

## Repository layout

```text
character_plugin/
  src/StudentController.cpp
  include/arkheon/character/ICharacterController.h
  tests/standalone_test.cpp
  character_plugin_200201852.dll
  build_windows.bat
  VERIFY_EXPORTS.txt

n8ro_visual_bridge/
  src/SimPlugin.cpp
  include/SimPlugin.h
  include/arkheon/character/ICharacterController.h
  payload/character_plugin_200201852.dll
  lua/human_animation_loop.lua
  sim-plugin.vcxproj

n8ro_mission/
  human_animation_loop.lua
  stubs/

evidence/
  rawan_bridge_latest_excerpt.log
  host.log
  app.log

video/
  Recording_vid.mp4
```

## Build the character DLL

Open an **x64 Native Tools Command Prompt for Visual Studio** inside `character_plugin/` and run:

```bat
build_windows.bat
```

Expected output:

```text
character_plugin_200201852.dll
```

The DLL exports:

```text
arkheon_character_sdk_version
arkheon_character_plugin_name
arkheon_character_get_motion_clips
arkheon_character_create
arkheon_character_destroy
arkheon_character_tick
```

## Install the DLL in N8RO

Copy the DLL to the expected character plugin location:

```text
D:\N8RO_2\bin\plugins\character\character_plugin_200201852.dll
```

The visual bridge also uses a payload copy:

```text
D:\N8RO_2\dev\samples\sim\sim-plugin\payload\character_plugin_200201852.dll
D:\N8RO_2\userPlugins\sim\rawan\character_plugin_200201852.dll
```

## Build/install the N8RO visual bridge

Copy the contents of `n8ro_visual_bridge/` into:

```text
D:\N8RO_2\dev\samples\sim\sim-plugin
```

Build in Visual Studio:

```text
Release | x64
Build > Rebuild Solution
```

The build deploys:

```text
D:\N8RO_2\userPlugins\sim\rawan-character-bridge.dll
D:\N8RO_2\userPlugins\sim\rawan\character_plugin_200201852.dll
```

## Mission script

Copy:

```text
n8ro_mission\human_animation_loop.lua
```

to:

```text
D:\N8RO_2\data\resources\missions\human_animation_loop.lua
```

The script cycles through the available N8RO/Nathan animation codes:

```lua
"Idle Walk Forward"
"Idle Shake"
"Idle Breathing"
"Idle Neutral"
```

The bridge maps these to:

```text
Idle Walk Forward -> WALK-LIKE
Idle Shake        -> PUSH-LIKE
Idle Breathing    -> CLIMB-LIKE
Idle Neutral      -> neutral/rest
```

## Runtime verification

After running N8RO, check:

```text
D:\N8RO_2\userPlugins\sim\rawan_bridge.log
```

Expected evidence:

```text
character DLL loaded: name='Rawan Character Plugin v0.1' sdk=0x00010000 clips=[12, 47, 83]
visual registration ok: Idle Shake -> PUSH-LIKE, Idle Breathing -> CLIMB-LIKE
state -> WALK-LIKE
state -> PUSH-LIKE
state -> CLIMB-LIKE
tick ok ... firstJointApply=1 visualRegistered=1
```

A captured runtime excerpt is included in:

```text
evidence/rawan_bridge_latest_excerpt.log
```

## Notes

The N8RO GLB viewer build used for testing exposes the Nathan model with the built-in animation codes `Idle Neutral`, `Idle Breathing`, `Idle Shake`, and `Idle Walk Forward`. The visual bridge registers the custom controller states through those available codes and verifies model-driven joint override output through runtime logs.
