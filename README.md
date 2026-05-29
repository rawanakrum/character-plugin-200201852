# Character Plugin 200201852

## Project Scope

This project follows the updated simplified Character Animation requirement.

The goal is to build a closed-library C++ model/plugin that determines 10 human joint-angle outputs and verifies the character motion workflow through the N8RO GLB viewer.

The project does not compute forces, torques, rigid-body dynamics, or contact physics. Motion is generated using joint-angle based kinematic control.

## Implemented Features

* Closed-source C++ DLL build
* Windows x64 Release build
* SDK version handshake: `0x00010000`
* Required exported functions verified with `dumpbin /EXPORTS`
* 10 major joint-angle outputs:

  * left/right shoulders, implemented through upper arm joints
  * left/right elbows, implemented through lower arm joints
  * left/right hips, implemented through thigh joints
  * left/right knees, implemented through calf/lower-leg joints
  * left/right ankles/feet, implemented through foot joints
* Procedural target joint rotations
* Quaternion normalization for stable joint output
* Smooth PD-like interpolation toward target joint angles
* Standalone test passing 1000 ticks
* Prepared Windows x64 DLL for N8RO integration
* Verified N8RO GLB viewer workflow with `GenericCivilianPresence`

## Build Steps

Open Developer PowerShell and run:

```powershell
cd "D:\project_CA_Plugin\character-plugin-200201852-main"
cmake -S . -B build -A x64
cmake --build build --config Release
```

## Output DLL

```text
build\Release\character_plugin_200201852.dll
```

A copy of the final DLL is also included in:

```text
submission\character_plugin_200201852.dll
```

## Required DLL Exports

The DLL exports the required character plugin functions:

```text
arkheon_character_create
arkheon_character_destroy
arkheon_character_get_motion_clips
arkheon_character_plugin_name
arkheon_character_sdk_version
arkheon_character_tick
```

These were verified using:

```powershell
dumpbin /EXPORTS "build\Release\character_plugin_200201852.dll"
```

## Standalone Test

The standalone test was compiled and run with:

```powershell
cl /std:c++17 /EHsc tests\standalone_test.cpp src\StudentController.cpp /I include
.\standalone_test.exe
```

Result:

```text
SDK version = 0x00010000
PASS: 1000 ticks, all quaternions finite & unit-ish
```

This confirms that the controller runs successfully and produces stable quaternion outputs.

## Joint-Angle Controller

The controller stores and updates 10 joint outputs using:

```cpp
joint_pose[ARK_JOINT_COUNT]
```

During each tick, it computes target local rotations, smooths the current pose toward the target pose, normalizes the quaternion, and writes the output to:

```cpp
out_overrides[i].local_rotation = c->joint_pose[i];
out_overrides[i].apply = 1;
```

This means the plugin produces one active local rotation output for each of the 10 required major joints.

## N8RO / GLB Viewer Verification

The N8RO application was used to run the `GenericCivilianPresence` scenario and verify the character motion canvas through the GLB viewer.

Workflow used:

1. Open N8RO.
2. Click the Active menu.
3. Open Scenario Editor and Simulation Control.
4. Load `GenericCivilianPresence`.
5. Apply the scenario.
6. Run the simulation.
7. Press `G`.
8. Open the GLB viewer.
9. Observe the human model and the configured 10-joint list.

The installed N8RO package did not expose a visible plugin-management panel or explicit plugin-load log. Therefore, DLL behavior was verified through the standalone test and export checks, while visual verification was performed in N8RO's GLB viewer according to the simplified workflow.

## Verification Screenshots

Additional proof screenshots are included in the `screenshots/` folder:

* `dumpbin_exports.png` shows the required DLL exports.
* `standalone_test_pass.png` shows the controller passing 1000 ticks with valid quaternions.
* `n8ro_glb_viewer_10_joints.png` shows the N8RO GLB viewer with the configured 10-joint list.

## Submission Contents

This repository includes:

```text
src/
include/
tests/
docs/
submission/
screenshots/
CMakeLists.txt
README.md
.gitignore
```

The final compiled DLL is located at:

```text
submission\character_plugin_200201852.dll
```
