# Design Document

## Updated Scope

This project follows the simplified joint-angle-only requirement.

The controller does not compute forces, torques, rigid-body dynamics, or contact physics. Instead, it generates target joint angles for the 10 required human joints and outputs them as local joint rotations.

## Controlled Joints

The plugin controls 10 major joint-angle outputs. The C++ SDK names these joints according to the driven bone segments, while the N8RO GLB viewer displays them using human-readable joint labels.

The mapping is:

1. left shoulder / upper arm
2. right shoulder / upper arm
3. left elbow / lower arm
4. right elbow / lower arm
5. left hip / thigh
6. right hip / thigh
7. left knee / calf/lower leg
8. right knee / calf/lower leg
9. left ankle / foot
10. right ankle / foot

Each joint output is represented as a local quaternion written to the `out_overrides` array.

## Motion Method

The controller uses procedural target poses for simple human-like motions. The target rotations are generated with axis-angle rotations and converted into quaternions.

The current joint pose is smoothed toward the target pose every tick using PD-like interpolation. After each update, the quaternion is normalized before being sent to the host.

This keeps the motion stable and prevents invalid quaternion output.

## Joint Angle Output

For every tick, the controller writes one rotation for each of the 10 major joints:

```cpp
out_overrides[i].local_rotation = c->joint_pose[i];
out_overrides[i].apply = 1;

## Stability

The controller normalizes every quaternion before sending it to the host. This prevents invalid rotation output such as NaN, infinity, or non-unit quaternions.

The plugin also catches exceptions inside the tick function and returns an error code instead of throwing across the DLL boundary.

## Testing and Verification

The controller was tested with the standalone test harness for 1000 ticks.

The test confirmed that all generated joint quaternions remain finite and unit-like.

The N8RO `GenericCivilianPresence` scenario and GLB viewer were used for visual verification of the character motion canvas and the configured 10-joint list.
