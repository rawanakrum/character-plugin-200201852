# Rawan Four Motion Character Animation Plugin

This repository contains my N8RO / ASTSIM character animation makeup project. The project implements a closed-library animation model for the NathanHuman GLB character and integrates it into the N8RO environment as a DLL plugin.

## Implemented motion states

The plugin registers and executes four motion states:

1. **Rawan Idle** — relaxed standing pose with subtle breathing/arm posture.
2. **Rawan Walk** — 8-phase gait cycle based on stance and swing phases.
3. **Rawan Push** — heavy-resistance pushing motion with braced knees, ankles, shoulders, and elbows.
4. **Rawan Climb** — vertical climbing-style reach and pull cycle.

The final demo mode cycles automatically through:

```text
Rawan Idle -> Rawan Walk -> Rawan Push -> Rawan Climb
```

## Ten-joint closed-library model

The closed-library model determines XYZ joint-angle targets for these 10 joints:

1. `leftHip`
2. `rightHip`
3. `leftKnee`
4. `rightKnee`
5. `leftAnkle`
6. `rightAnkle`
7. `leftShoulder`
8. `rightShoulder`
9. `leftElbow`
10. `rightElbow`

During N8RO viewer testing, direct hip authoring on NathanHuman caused a persistent leg-in-air rig artifact. To keep the final GLB viewer motion physically coherent, the model still computes the hip angles internally, but the N8RO output adapter hard-blocks direct hip writes. The visible viewer output is therefore stabilized through knees, ankles, shoulders, and elbows while preserving the 10-joint closed-library calculation in the motion model and standalone test.

## Repository contents

```text
RawanFourMotionPlugin/          C++ DLL plugin source
mission/                        N8RO Lua mission helper
standalone_test/                Standalone C++ test with no N8RO dependency
demo/                           Screen capture video evidence
logs/                           Example plugin runtime log
docs/                           Design and submission notes
visual_studio/                  Visual Studio project files
```

## How to run in N8RO

1. Build the Visual Studio DLL project using the N8RO / ASTSIM SDK include and library paths.
2. Copy the built DLL into the N8RO user plugin folder.
3. Copy `mission/human_animation_loop.lua` into the relevant scenario / mission script location.
4. In the N8RO Scenario Editor Animation tab, set the default animation to:

```text
Rawan Idle
```

5. Keep **Calib First OFF** for the final demo. In testing, Calib First ON kept the character closer to a T-pose and interfered with the intended plugin animation.
6. Run the GLB viewer and verify the automatic four-state sequence.

Expected log marker:

```text
version: 3.6.5-climb-push-action-matrices
```

## Standalone test

The standalone test is dependency-free and matches the Linux sanity-test style used for final verification. It simulates 1000 ticks of the closed-library motion model, converts the 10 computed joint-angle vectors into quaternions, and verifies that every quaternion is finite and approximately unit length.

### Linux / macOS

```bash
cd standalone_test
./run_test.sh
```

### Windows Developer Command Prompt

```bat
cd standalone_test
run_test.bat
```

Expected output:

```text
Linux standalone sanity test:
SDK version = 0x00010000
PASS: 1000 ticks, all quaternions finite & unit-ish
```

See also:

```text
standalone_test/sample_output.txt
```

