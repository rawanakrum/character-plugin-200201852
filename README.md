# Character Plugin 200201852

## Project Scope

This project follows the updated simplified Character Animation requirement.

The goal is to build a closed-library C++ model/plugin that determines 10 human joint angles and verifies the character motion through the N8RO GLB viewer.

The project does not compute forces, rigid-body dynamics, or contact physics. Motion is generated using joint-angle based kinematic control.

## Implemented Features

- Closed-source C++ DLL build
- Windows x64 Release build
- SDK version handshake: `0x00010000`
- Required exported functions verified with `dumpbin /EXPORTS`
- 10 major joint-angle outputs:
  - left/right shoulders, implemented through upper arm joints
  - left/right elbows, implemented through lower arm joints
  - left/right hips, implemented through thigh joints
  - left/right knees, implemented through calf/lower-leg joints
  - left/right ankles/feet, implemented through foot joints
- Procedural target joint rotations
- Quaternion normalization for stable joint output
- Smooth PD-like interpolation toward target joint angles
- Standalone test passing 1000 ticks
- Prepared Windows x64 DLL for N8RO integration
- Verified N8RO GLB viewer workflow with GenericCivilianPresence

## Build Steps

Open Developer PowerShell and run:

```powershell
cd C:\project_CA_Plugin\character-plugin-200201852-main
cmake -S . -B build -A x64
cmake --build build --config Release

