# Submission Description

## Project summary

This project implements a closed-library character animation plugin for the N8RO NathanHuman GLB viewer. The plugin registers four named animation states and computes joint-angle motion procedurally using C++.

## Motion states implemented

### 1. Rawan Idle
A stable standing pose with relaxed shoulders and elbows. The idle state is used as the initial/default animation for the mission script.

### 2. Rawan Walk
An 8-phase gait-style walking cycle. The model follows stance and swing phase timing and computes coordinated joint angles for the hips, knees, ankles, shoulders, and elbows.

### 3. Rawan Push
A pushing motion state based on a heavy-resistance force profile. The pose uses braced knees, ankle support, arms forward, and elbow stabilization to visually communicate pushing.

### 4. Rawan Climb
A climbing motion state based on a vertical ascent / reach-pull profile. The pose alternates between high-step and body-ascent phases with coordinated arm and leg movement.

## 10 joints determined by the model

The closed-library model computes XYZ angle targets for these 10 joints:

1. leftHip
2. rightHip
3. leftKnee
4. rightKnee
5. leftAnkle
6. rightAnkle
7. leftShoulder
8. rightShoulder
9. leftElbow
10. rightElbow

## Stability note

The NathanHuman GLB rig exposed hip joints that produced unstable leg lifting when direct hip overrides were authored into the viewer. For the final stable integration, the model still determines hip angles internally, but the N8RO output writer hard-blocks direct hip authoring. This was done to keep the final character motion meaningful, physically coherent, and stable in the GLB viewer.

## Evidence included

- Plugin source code in `RawanFourMotionPlugin/`
- Mission Lua script in `mission/`
- Standalone C++ test in `standalone_test/`
- Example standalone output in `standalone_test/sample_output.txt`
- Demo video in `demo/recording4.mp4`
- Runtime log in `logs/rawan-four-motion-plugin.log`
