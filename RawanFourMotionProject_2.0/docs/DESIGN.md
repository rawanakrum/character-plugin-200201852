# Design Notes

## Closed-library model

The motion model is implemented in C++ and computes joint-angle targets internally. The model does not use forces, dynamics, collision simulation, or contact physics. It only outputs joint angles.

## FK / IK-style approach

The final system uses forward-kinematics-style authored joint angles. For walking, pushing, and climbing, the motion tables are organized as timed keyframes. The system interpolates between keyframes to create continuous motion.

The project also uses IK-style reasoning for support/contact phases. For example, the walking state separates stance and swing phases, pushing uses braced support poses, and climbing alternates reach/pull phases. This is implemented as angle timing and constraints rather than a full physics solver.

## State separation

Each state has separate motion logic:

- Idle
- Walk
- Push
- Climb

The states do not share a common idle helper that could accidentally overwrite another motion state's pose. This makes the final behavior easier to debug and tune.

## Viewer safety layer

The NathanHuman rig's hip joints were unstable when directly overridden. The final design therefore separates the closed-library calculation from the N8RO viewer authoring layer:

- The closed-library model computes all 10 joints.
- The viewer adapter writes only stable joint overrides.
- Hip writes are hard-blocked to prevent the leg-in-air artifact.

This keeps the final output meaningful and physically coherent in the GLB viewer.
