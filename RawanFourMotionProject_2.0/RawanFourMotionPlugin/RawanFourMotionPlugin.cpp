// Rawan Character Animation Makeup Project
// N8RO NathanHuman animation plugin.
//
// Version 3.6.5-climb-push-action-matrices
// Purpose of this pass:
// - Closed-library model computes 10 joint angles for each visible state.
// - Hard hip restraints remain active while Push/Climb use dedicated 3D action matrices.
// - The user's Walk, Push, and Climb XYZ tables are computed as 10-joint closed-library models, then remapped through restrained Nathan-safe output.
// - Shoulder base offset is preserved during walk so arms do not collapse into T-pose.
// - Keep separate state functions for Idle / Walk / Push / Climb.

#include "RawanFourMotionPlugin.h"

#include <model/AnimationModel.h>
#include <model/ModelFactoryRegistry.h>
#include <plugin/IModelPluginService.h>
#include <plugin/PluginContext.h>
#include <plugin/IPluginServices.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace arkheon::sample::rawanfourmotion {
namespace {

constexpr const char* kModelType = "animationModelNathanHuman";
constexpr const char* kRawanIdleCode = "Rawan Idle";
constexpr const char* kRawanWalkCode = "Rawan Walk";
constexpr const char* kRawanPushCode = "Rawan Push";
constexpr const char* kRawanClimbCode = "Rawan Climb";

constexpr double kPi = 3.14159265358979323846;
constexpr double kStateSeconds = 5.0;

// FINAL DEMO: true  = any requested Rawan animation displays the internal
//                     Idle -> Walk -> Push -> Climb cycle.
// TUNING:     false = the requested animation stays isolated, so selecting
//                     "Rawan Walk" only evaluates the Walk pose, etc.
constexpr bool kAutoCycleDemoMode = true;

// Nathan's exposed hip joints behave like rig/root controls in this viewer.
// 3.6.0 full XYZ and 3.6.2 soft hip authoring both caused the leg-in-air bug,
// so this build restores hard restraints: hips may be computed internally, but
// they are never authored into the GLB viewer output.
constexpr bool kAuthorHipOverrides = false;
constexpr bool kHardBlockHipOverrides = true;

// These remain only for the internal closed-library calculation and logs.
// They are NOT used to author hip output while kHardBlockHipOverrides is true.
constexpr double kWalkHipScaleX = 0.18;
constexpr double kWalkHipScaleY = 0.30;
constexpr double kWalkHipScaleZ = 0.35;

constexpr double kHipOutputMinX = -8.0;
constexpr double kHipOutputMaxX =  8.0;
constexpr double kHipOutputMinY = -3.5;
constexpr double kHipOutputMaxY =  3.5;
constexpr double kHipOutputMinZ = -6.0;
constexpr double kHipOutputMaxZ =  6.0;

// Extra lower-limb restraints. Full gait-table knee/ankle values were too large
// for Nathan's visible joint basis, so final authored angles are clamped to a
// stable, readable range.
constexpr double kKneeOutputMinZ = -28.0;
constexpr double kKneeOutputMaxZ =  -4.0;
constexpr double kAnkleOutputMinZ = -7.0;
constexpr double kAnkleOutputMaxZ =  7.0;


struct PoseDeg {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct WalkPose3D {
    PoseDeg leftHip;
    PoseDeg rightHip;
    PoseDeg leftKnee;
    PoseDeg rightKnee;
    PoseDeg leftAnkle;
    PoseDeg rightAnkle;
    PoseDeg leftShoulder;
    PoseDeg rightShoulder;
    PoseDeg leftElbow;
    PoseDeg rightElbow;
};

struct WalkKeyframe3D {
    double phase = 0.0; // 0.0 to 1.0
    WalkPose3D pose;
};

struct ActionKeyframe3D {
    double phase = 0.0; // 0.0 to 1.0
    WalkPose3D pose;
};

enum class ActionPoseMode {
    Push,
    Climb
};

enum class VisibleState {
    Idle,
    Walk,
    Push,
    Climb
};

[[nodiscard]] double deg(double degrees) {
    return degrees * kPi / 180.0;
}

[[nodiscard]] double clamp01(double v) {
    if (v < 0.0) return 0.0;
    if (v > 1.0) return 1.0;
    return v;
}

[[nodiscard]] double smooth01(double v) {
    v = clamp01(v);
    return v * v * (3.0 - 2.0 * v);
}

[[nodiscard]] double wrap01(double v) {
    v = std::fmod(v, 1.0);
    if (v < 0.0) v += 1.0;
    return v;
}

[[nodiscard]] double phasePulse(double phase, double center, double halfWidth) {
    phase = wrap01(phase);
    center = wrap01(center);
    double distance = std::abs(phase - center);
    distance = std::min(distance, 1.0 - distance);
    if (distance >= halfWidth) {
        return 0.0;
    }
    return smooth01(1.0 - distance / halfWidth);
}

[[nodiscard]] double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

[[nodiscard]] PoseDeg lerpPose(const PoseDeg& a, const PoseDeg& b, double t) {
    t = smooth01(t);
    return {lerp(a.x, b.x, t), lerp(a.y, b.y, t), lerp(a.z, b.z, t)};
}

[[nodiscard]] double cyclicKeyframe5(
    double phase,
    double t0, double v0,
    double t1, double v1,
    double t2, double v2,
    double t3, double v3,
    double t4, double v4) {
    phase = wrap01(phase);

    auto segment = [](double p, double a, double b, double va, double vb) {
        const double u = smooth01((p - a) / (b - a));
        return lerp(va, vb, u);
    };

    if (phase < t1) return segment(phase, t0, t1, v0, v1);
    if (phase < t2) return segment(phase, t1, t2, v1, v2);
    if (phase < t3) return segment(phase, t2, t3, v2, v3);
    if (phase < t4) return segment(phase, t3, t4, v3, v4);

    // Wrap final segment from t4 back to 1.0/t0.
    const double u = smooth01((phase - t4) / (1.0 - t4));
    return lerp(v4, v0, u);
}


[[nodiscard]] WalkPose3D interpolateWalkPose3D(const WalkPose3D& a, const WalkPose3D& b, double t) {
    t = smooth01(t);
    return {
        lerpPose(a.leftHip, b.leftHip, t),
        lerpPose(a.rightHip, b.rightHip, t),
        lerpPose(a.leftKnee, b.leftKnee, t),
        lerpPose(a.rightKnee, b.rightKnee, t),
        lerpPose(a.leftAnkle, b.leftAnkle, t),
        lerpPose(a.rightAnkle, b.rightAnkle, t),
        lerpPose(a.leftShoulder, b.leftShoulder, t),
        lerpPose(a.rightShoulder, b.rightShoulder, t),
        lerpPose(a.leftElbow, b.leftElbow, t),
        lerpPose(a.rightElbow, b.rightElbow, t)
    };
}

[[nodiscard]] WalkPose3D computeEightPhaseWalkPose3D(double phase) {
    // Full 10-joint / 3-axis gait table adapted from the user's XYZ reference.
    // X = sagittal flexion/extension, Y = transverse rotation, Z = frontal ab/adduction.
    static constexpr WalkKeyframe3D kWalkTimeline[] = {
        {0.00, { // Phase 1: Initial Contact / Heel Strike
            {+30.0, +4.0, -2.0}, {-10.0, -4.0, +1.0},
            {  0.0,  0.0,  0.0}, {+40.0, +2.0,  0.0},
            {  0.0, -3.0,  0.0}, {-20.0, +1.0, -2.0},
            {-15.0, -2.0, +5.0}, {+15.0, +2.0, +5.0},
            {+20.0,  0.0,  0.0}, {+20.0,  0.0,  0.0}
        }},
        {0.12, { // Phase 2: Loading Response / Foot Flat
            {+25.0, +2.0, -4.0}, {+15.0, -2.0, +2.0},
            {+15.0, +1.0, -1.0}, {+60.0, +3.0, +1.0},
            { -5.0, -2.0, +2.0}, {-10.0,  0.0, -1.0},
            {-10.0, -1.0, +4.0}, {+10.0, +1.0, +6.0},
            {+22.0,  0.0,  0.0}, {+18.0,  0.0,  0.0}
        }},
        {0.31, { // Phase 3: Mid-Stance
            {  0.0,  0.0,  0.0}, {+25.0,  0.0,  0.0},
            { +5.0,  0.0,  0.0}, {+30.0, +1.0,  0.0},
            { +8.0,  0.0, +1.0}, {  0.0, -1.0, -1.0},
            {  0.0,  0.0, +3.0}, {  0.0,  0.0, +3.0},
            {+25.0,  0.0,  0.0}, {+15.0,  0.0,  0.0}
        }},
        {0.50, { // Phase 4: Terminal Stance / Heel Off
            {-15.0, -4.0, +1.0}, {+30.0, +4.0, -2.0},
            {  0.0, -2.0,  0.0}, {  0.0,  0.0,  0.0},
            {+12.0, +1.0, -2.0}, {  0.0, -3.0,  0.0},
            {+15.0, +2.0, +5.0}, {-15.0, -2.0, +5.0},
            {+15.0,  0.0,  0.0}, {+20.0,  0.0,  0.0}
        }},
        {0.60, { // Phase 5: Pre-Swing / Toe-Off
            {-10.0, -4.0, +1.0}, {+30.0, +4.0, -2.0},
            {+40.0, +2.0,  0.0}, {  0.0,  0.0,  0.0},
            {-20.0, +1.0, -2.0}, {  0.0, -3.0,  0.0},
            {+15.0, +2.0, +5.0}, {-15.0, -2.0, +5.0},
            {+20.0,  0.0,  0.0}, {+20.0,  0.0,  0.0}
        }},
        {0.75, { // Phase 6: Initial Swing
            {+15.0, -2.0, +2.0}, {+25.0, +2.0, -4.0},
            {+60.0, +3.0, +1.0}, {+15.0, +1.0, -1.0},
            {-10.0,  0.0, -1.0}, { -5.0, -2.0, +2.0},
            {+10.0, +1.0, +6.0}, {-10.0, -1.0, +4.0},
            {+18.0,  0.0,  0.0}, {+22.0,  0.0,  0.0}
        }},
        {0.87, { // Phase 7: Mid-Swing
            {+25.0,  0.0,  0.0}, {  0.0,  0.0,  0.0},
            {+30.0, +1.0,  0.0}, { +5.0,  0.0,  0.0},
            {  0.0, -1.0, -1.0}, { +8.0,  0.0, +1.0},
            {  0.0,  0.0, +3.0}, {  0.0,  0.0, +3.0},
            {+15.0,  0.0,  0.0}, {+25.0,  0.0,  0.0}
        }},
        {1.00, { // Phase 8: Terminal Swing / Next Heel Strike
            {+30.0, +4.0, -2.0}, {-10.0, -4.0, +1.0},
            {  0.0,  0.0,  0.0}, {+40.0, +2.0,  0.0},
            {  0.0, -3.0,  0.0}, {-20.0, +1.0, -2.0},
            {-15.0, -2.0, +5.0}, {+15.0, +2.0, +5.0},
            {+20.0,  0.0,  0.0}, {+20.0,  0.0,  0.0}
        }}
    };

    phase = wrap01(phase);
    for (int i = 0; i < 7; ++i) {
        const WalkKeyframe3D& lower = kWalkTimeline[i];
        const WalkKeyframe3D& upper = kWalkTimeline[i + 1];
        if (phase >= lower.phase && phase <= upper.phase) {
            const double span = upper.phase - lower.phase;
            const double u = (span > 0.0) ? (phase - lower.phase) / span : 0.0;
            return interpolateWalkPose3D(lower.pose, upper.pose, u);
        }
    }

    return kWalkTimeline[7].pose;
}

[[nodiscard]] double clampDeg(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
}

[[nodiscard]] PoseDeg mapGaitHipToNathanRig(const PoseDeg& gaitHip) {
    // Do not copy anatomical XYZ directly to Nathan. First keep the anatomical
    // gait inside human walking limits, then scale the result to Nathan's more
    // sensitive local hip axes. This gives visible pelvis/hip motion without
    // repeating the 3.6.0 raised-leg glitch.
    auto clip = [](double value, double lo, double hi) {
        return std::max(lo, std::min(hi, value));
    };

    const double anatomicalX = clip(gaitHip.x, -20.0, 45.0);
    const double anatomicalY = clip(gaitHip.y, -10.0, 10.0);
    const double anatomicalZ = clip(gaitHip.z, -10.0, 15.0);

    return {
        clip(anatomicalX * kWalkHipScaleX, kHipOutputMinX, kHipOutputMaxX),
        clip(anatomicalY * kWalkHipScaleY, kHipOutputMinY, kHipOutputMaxY),
        clip(anatomicalZ * kWalkHipScaleZ, kHipOutputMinZ, kHipOutputMaxZ)
    };
}

[[nodiscard]] PoseDeg mapGaitKneeToNathanRig(const PoseDeg& gaitKnee) {
    // In the Nathan viewer, knee flexion has been observed on local Z, not
    // the anatomical X axis from the reference table. The full 60-degree gait
    // value was too aggressive, so final output is softened and clamped.
    const double flex = std::max(0.0, gaitKnee.x);
    const double z = clampDeg(-4.0 - 0.38 * flex, kKneeOutputMinZ, kKneeOutputMaxZ);
    return {0.0, 0.0, z};
}

[[nodiscard]] PoseDeg mapGaitAnkleToNathanRig(const PoseDeg& gaitAnkle) {
    // Reference ankle X uses dorsiflexion (+) and plantarflexion (-). Nathan's
    // visible toe/ankle motion is safer as a small restrained local-Z correction.
    const double z = clampDeg(-0.30 * gaitAnkle.x, kAnkleOutputMinZ, kAnkleOutputMaxZ);
    return {0.0, 0.0, z};
}

[[nodiscard]] PoseDeg mapGaitShoulderToNathanRig(const PoseDeg& gaitShoulder) {
    // Critical rig offset: Nathan needs shoulder Y around 80 degrees for arms
    // to hang naturally. The gait table's shoulder XYZ values are anatomical
    // deltas, not absolute rig rotations.
    const double swingZ = -1.15 * gaitShoulder.x;
    const double liftX = -20.0 + 0.15 * gaitShoulder.z;
    return {liftX, 80.0, swingZ};
}

[[nodiscard]] PoseDeg mapGaitElbowToNathanRig(const PoseDeg& gaitElbow) {
    const double flex = std::max(0.0, gaitElbow.x);
    return {0.0, 0.0, -11.0 - 0.25 * flex};
}

[[nodiscard]] WalkPose3D mapGaitPoseToNathanRig(const WalkPose3D& gaitPose) {
    return {
        mapGaitHipToNathanRig(gaitPose.leftHip),
        mapGaitHipToNathanRig(gaitPose.rightHip),
        mapGaitKneeToNathanRig(gaitPose.leftKnee),
        mapGaitKneeToNathanRig(gaitPose.rightKnee),
        mapGaitAnkleToNathanRig(gaitPose.leftAnkle),
        mapGaitAnkleToNathanRig(gaitPose.rightAnkle),
        mapGaitShoulderToNathanRig(gaitPose.leftShoulder),
        mapGaitShoulderToNathanRig(gaitPose.rightShoulder),
        mapGaitElbowToNathanRig(gaitPose.leftElbow),
        mapGaitElbowToNathanRig(gaitPose.rightElbow)
    };
}


[[nodiscard]] WalkPose3D interpolateActionPose3D(const ActionKeyframe3D* timeline, int count, double phase) {
    phase = wrap01(phase);
    for (int i = 0; i < count - 1; ++i) {
        const ActionKeyframe3D& lower = timeline[i];
        const ActionKeyframe3D& upper = timeline[i + 1];
        if (phase >= lower.phase && phase <= upper.phase) {
            const double span = upper.phase - lower.phase;
            const double u = (span > 0.0) ? (phase - lower.phase) / span : 0.0;
            return interpolateWalkPose3D(lower.pose, upper.pose, u);
        }
    }
    return timeline[count - 1].pose;
}

[[nodiscard]] WalkPose3D computePushPose3D(double phase) {
    // Four-stage heavy-resistance push matrix adapted from the user's action matrix.
    // Hips are still computed as part of the closed-library 10-joint model, but
    // the final writer-level hip restraint prevents Nathan's leg-pop glitch.
    static constexpr ActionKeyframe3D kPushTimeline[] = {
        {0.00, { // Stage 1: left leg drive / max extension force
            {-20.0, -4.0, -2.0}, {+40.0, +4.0, +4.0},
            { +5.0,  0.0,  0.0}, {+50.0,  0.0,  0.0},
            {-20.0, -2.0,  0.0}, {+15.0, +2.0,  0.0},
            {+45.0, -5.0, +10.0}, {+45.0, +5.0, +10.0},
            {+25.0,  0.0,  0.0}, {+25.0,  0.0,  0.0}
        }},
        {0.25, { // Stage 2: transition / rear leg comes forward
            {+20.0,  0.0, +2.0}, {+10.0, +2.0,  0.0},
            {+40.0,  0.0,  0.0}, {+25.0,  0.0,  0.0},
            {  0.0,  0.0,  0.0}, { -5.0,  0.0,  0.0},
            {+40.0, -3.0, +8.0}, {+40.0, +3.0, +8.0},
            {+35.0,  0.0,  0.0}, {+35.0,  0.0,  0.0}
        }},
        {0.50, { // Stage 3: right leg drive / max extension force
            {+40.0, -4.0, +4.0}, {-20.0, +4.0, -2.0},
            {+50.0,  0.0,  0.0}, { +5.0,  0.0,  0.0},
            {+15.0, -2.0,  0.0}, {-20.0, +2.0,  0.0},
            {+45.0, +5.0, +10.0}, {+45.0, -5.0, +10.0},
            {+25.0,  0.0,  0.0}, {+25.0,  0.0,  0.0}
        }},
        {0.75, { // Stage 4: transition / left leg comes forward
            {+10.0, -2.0,  0.0}, {+20.0,  0.0, +2.0},
            {+25.0,  0.0,  0.0}, {+40.0,  0.0,  0.0},
            { -5.0,  0.0,  0.0}, {  0.0,  0.0,  0.0},
            {+40.0, +3.0, +8.0}, {+40.0, -3.0, +8.0},
            {+35.0,  0.0,  0.0}, {+35.0,  0.0,  0.0}
        }},
        {1.00, { // Wrap
            {-20.0, -4.0, -2.0}, {+40.0, +4.0, +4.0},
            { +5.0,  0.0,  0.0}, {+50.0,  0.0,  0.0},
            {-20.0, -2.0,  0.0}, {+15.0, +2.0,  0.0},
            {+45.0, -5.0, +10.0}, {+45.0, +5.0, +10.0},
            {+25.0,  0.0,  0.0}, {+25.0,  0.0,  0.0}
        }}
    };
    return interpolateActionPose3D(kPushTimeline, 5, phase);
}

[[nodiscard]] WalkPose3D computeClimbPose3D(double phase) {
    // Four-stage vertical climbing matrix adapted from the user's action matrix.
    static constexpr ActionKeyframe3D kClimbTimeline[] = {
        {0.00, { // Stage 1: left foot high step up / right leg weight-bearing
            {+75.0, +5.0, +10.0}, {-10.0, -3.0, -2.0},
            {+90.0,  0.0,   0.0}, {+10.0,  0.0,  0.0},
            {+15.0,  0.0,   0.0}, {-15.0,  0.0,  0.0},
            {+60.0, +10.0, +20.0}, {-20.0,  -5.0, +10.0},
            {+30.0,   0.0,  0.0}, {+90.0,   0.0,  0.0}
        }},
        {0.25, { // Stage 2: left leg drive / body ascent
            {+35.0, +2.0, +4.0}, {+40.0, -2.0, +5.0},
            {+45.0,  0.0,  0.0}, {+70.0,  0.0,  0.0},
            {  0.0,  0.0,  0.0}, { +5.0,  0.0,  0.0},
            {+20.0, +5.0, +15.0}, {+40.0, +10.0, +15.0},
            {+75.0,  0.0,  0.0}, {+45.0,  0.0,  0.0}
        }},
        {0.50, { // Stage 3: right foot high step up / left leg weight-bearing
            {-10.0, +3.0, -2.0}, {+75.0, -5.0, +10.0},
            {+10.0,  0.0,  0.0}, {+90.0,  0.0,   0.0},
            {-15.0,  0.0,  0.0}, {+15.0,  0.0,   0.0},
            {-20.0, +5.0, +10.0}, {+60.0, -10.0, +20.0},
            {+90.0,  0.0,  0.0}, {+30.0,   0.0,  0.0}
        }},
        {0.75, { // Stage 4: right leg drive / body ascent
            {+40.0, +2.0, +5.0}, {+35.0, -2.0, +4.0},
            {+70.0,  0.0,  0.0}, {+45.0,  0.0,  0.0},
            { +5.0,  0.0,  0.0}, {  0.0,  0.0,  0.0},
            {+40.0, -10.0, +15.0}, {+20.0, -5.0, +15.0},
            {+45.0,   0.0,  0.0}, {+75.0,  0.0,  0.0}
        }},
        {1.00, { // Wrap
            {+75.0, +5.0, +10.0}, {-10.0, -3.0, -2.0},
            {+90.0,  0.0,   0.0}, {+10.0,  0.0,  0.0},
            {+15.0,  0.0,   0.0}, {-15.0,  0.0,  0.0},
            {+60.0, +10.0, +20.0}, {-20.0,  -5.0, +10.0},
            {+30.0,   0.0,  0.0}, {+90.0,   0.0,  0.0}
        }}
    };
    return interpolateActionPose3D(kClimbTimeline, 5, phase);
}

[[nodiscard]] PoseDeg mapActionHipToNathanRig(const PoseDeg& actionHip, ActionPoseMode mode) {
    // Retained for the closed-library 10-joint model. It produces small, limited
    // hip cues, but addRawJointDeg still hard-blocks hip output for Nathan.
    const double xLo = (mode == ActionPoseMode::Climb) ? -20.0 : -30.0;
    const double xHi = (mode == ActionPoseMode::Climb) ?  85.0 :  50.0;
    const double yLo = (mode == ActionPoseMode::Climb) ? -10.0 : -15.0;
    const double yHi = (mode == ActionPoseMode::Climb) ?  10.0 :  15.0;
    const double zLo = (mode == ActionPoseMode::Climb) ?  -5.0 : -10.0;
    const double zHi = (mode == ActionPoseMode::Climb) ?  15.0 :  10.0;

    const double sx = (mode == ActionPoseMode::Climb) ? 0.10 : 0.14;
    const double sy = (mode == ActionPoseMode::Climb) ? 0.18 : 0.20;
    const double sz = (mode == ActionPoseMode::Climb) ? 0.22 : 0.25;

    return {
        clampDeg(clampDeg(actionHip.x, xLo, xHi) * sx, kHipOutputMinX, kHipOutputMaxX),
        clampDeg(clampDeg(actionHip.y, yLo, yHi) * sy, kHipOutputMinY, kHipOutputMaxY),
        clampDeg(clampDeg(actionHip.z, zLo, zHi) * sz, kHipOutputMinZ, kHipOutputMaxZ)
    };
}

[[nodiscard]] PoseDeg mapActionKneeToNathanRig(const PoseDeg& actionKnee, ActionPoseMode mode) {
    const double flex = std::max(0.0, actionKnee.x);
    const double scale = (mode == ActionPoseMode::Climb) ? 0.30 : 0.34;
    const double z = clampDeg(-5.0 - scale * flex, kKneeOutputMinZ, kKneeOutputMaxZ);
    return {0.0, 0.0, z};
}

[[nodiscard]] PoseDeg mapActionAnkleToNathanRig(const PoseDeg& actionAnkle, ActionPoseMode mode) {
    const double scale = (mode == ActionPoseMode::Climb) ? -0.28 : -0.30;
    const double z = clampDeg(scale * actionAnkle.x, kAnkleOutputMinZ, kAnkleOutputMaxZ);
    return {0.0, 0.0, z};
}

[[nodiscard]] PoseDeg mapPushShoulderToNathanRig(const PoseDeg& actionShoulder, bool leftSide) {
    // Heavy push: preserve a stable forward-hand contact pose. The matrix adds
    // small pressure/compression variations but does not replace the rig offset.
    const double side = leftSide ? -1.0 : 1.0;
    const double x = 8.0 + 0.07 * actionShoulder.x;
    const double y = 64.0 + 0.25 * actionShoulder.y;
    const double z = -72.0 + side * 0.20 * actionShoulder.z;
    return {clampDeg(x, 7.0, 14.0), clampDeg(y, 58.0, 70.0), clampDeg(z, -78.0, -66.0)};
}

[[nodiscard]] PoseDeg mapClimbShoulderToNathanRig(const PoseDeg& actionShoulder, bool leftSide) {
    // Vertical climb: actionShoulder.x expresses reach height. Remap it to a
    // Nathan-safe high-arm pose while keeping the shoulder away from T-pose.
    static_cast<void>(leftSide);
    const double reach = clamp01((actionShoulder.x + 20.0) / 80.0);
    const double x = -18.0 - 10.0 * reach + 0.05 * actionShoulder.z;
    const double y = 72.0 - 48.0 * reach + 0.15 * actionShoulder.y;
    const double z = -42.0 - 42.0 * reach + 0.12 * actionShoulder.z;
    return {clampDeg(x, -30.0, -12.0), clampDeg(y, 22.0, 74.0), clampDeg(z, -86.0, -36.0)};
}

[[nodiscard]] PoseDeg mapActionElbowToNathanRig(const PoseDeg& actionElbow, ActionPoseMode mode) {
    const double flex = std::max(0.0, actionElbow.x);
    if (mode == ActionPoseMode::Climb) {
        return {0.0, clampDeg(-0.42 * flex, -42.0, -10.0), clampDeg(-9.0 - 0.06 * flex, -16.0, -9.0)};
    }
    return {0.0, -10.0, clampDeg(-20.0 - 0.12 * flex, -26.0, -20.0)};
}

[[nodiscard]] WalkPose3D mapPushPoseToNathanRig(const WalkPose3D& actionPose) {
    return {
        mapActionHipToNathanRig(actionPose.leftHip, ActionPoseMode::Push),
        mapActionHipToNathanRig(actionPose.rightHip, ActionPoseMode::Push),
        mapActionKneeToNathanRig(actionPose.leftKnee, ActionPoseMode::Push),
        mapActionKneeToNathanRig(actionPose.rightKnee, ActionPoseMode::Push),
        mapActionAnkleToNathanRig(actionPose.leftAnkle, ActionPoseMode::Push),
        mapActionAnkleToNathanRig(actionPose.rightAnkle, ActionPoseMode::Push),
        mapPushShoulderToNathanRig(actionPose.leftShoulder, true),
        mapPushShoulderToNathanRig(actionPose.rightShoulder, false),
        mapActionElbowToNathanRig(actionPose.leftElbow, ActionPoseMode::Push),
        mapActionElbowToNathanRig(actionPose.rightElbow, ActionPoseMode::Push)
    };
}

[[nodiscard]] WalkPose3D mapClimbPoseToNathanRig(const WalkPose3D& actionPose) {
    return {
        mapActionHipToNathanRig(actionPose.leftHip, ActionPoseMode::Climb),
        mapActionHipToNathanRig(actionPose.rightHip, ActionPoseMode::Climb),
        mapActionKneeToNathanRig(actionPose.leftKnee, ActionPoseMode::Climb),
        mapActionKneeToNathanRig(actionPose.rightKnee, ActionPoseMode::Climb),
        mapActionAnkleToNathanRig(actionPose.leftAnkle, ActionPoseMode::Climb),
        mapActionAnkleToNathanRig(actionPose.rightAnkle, ActionPoseMode::Climb),
        mapClimbShoulderToNathanRig(actionPose.leftShoulder, true),
        mapClimbShoulderToNathanRig(actionPose.rightShoulder, false),
        mapActionElbowToNathanRig(actionPose.leftElbow, ActionPoseMode::Climb),
        mapActionElbowToNathanRig(actionPose.rightElbow, ActionPoseMode::Climb)
    };
}


[[nodiscard]] const char* visibleStateName(VisibleState state) {
    switch (state) {
    case VisibleState::Idle: return kRawanIdleCode;
    case VisibleState::Walk: return kRawanWalkCode;
    case VisibleState::Push: return kRawanPushCode;
    case VisibleState::Climb: return kRawanClimbCode;
    default: return "Unknown";
    }
}

[[nodiscard]] VisibleState stateFromRequestedAnimation(const char* requestedAnimationCode) {
    if (requestedAnimationCode == nullptr) {
        return VisibleState::Idle;
    }

    const std::string requested(requestedAnimationCode);
    if (requested == kRawanWalkCode) return VisibleState::Walk;
    if (requested == kRawanPushCode) return VisibleState::Push;
    if (requested == kRawanClimbCode) return VisibleState::Climb;
    return VisibleState::Idle;
}

[[nodiscard]] const char* animationModeName() {
    return kAutoCycleDemoMode ? "auto-cycle demo" : "isolated tuning";
}

[[nodiscard]] std::string pluginLogPath() {
#ifdef _WIN32
    char* pluginDir = nullptr;
    size_t pluginDirLength = 0;

    if (_dupenv_s(&pluginDir, &pluginDirLength, "N8RO_RELEASE_USER_SIM_PLUGINS") == 0 &&
        pluginDir != nullptr &&
        *pluginDir != '\0') {
        std::string path(pluginDir);
        std::free(pluginDir);

        const char last = path.empty() ? '\0' : path.back();
        if (last != '\\' && last != '/') {
            path += '\\';
        }
        path += "rawan-four-motion-plugin.log";
        return path;
    }

    if (pluginDir != nullptr) {
        std::free(pluginDir);
    }
#else
    const char* pluginDir = std::getenv("N8RO_RELEASE_USER_SIM_PLUGINS");
    if (pluginDir && *pluginDir != '\0') {
        std::string path(pluginDir);
        const char last = path.empty() ? '\0' : path.back();
        if (last != '\\' && last != '/') {
            path += '/';
        }
        path += "rawan-four-motion-plugin.log";
        return path;
    }
#endif

    return "D:\\N8RO_2\\userPlugins\\sim\\rawan-four-motion-plugin.log";
}

void logLine(const std::string& message) {
    static std::mutex logMutex;
    std::lock_guard<std::mutex> lock(logMutex);

    std::ofstream log(pluginLogPath(), std::ios::app);
    if (log.is_open()) {
        log << message << '\n';
    }
}

[[nodiscard]] const char* yesNo(bool value) {
    return value ? "yes" : "no";
}

[[nodiscard]] double relativeDemoTime(double simulationTimeSeconds) {
    static std::mutex cycleMutex;
    static bool startCaptured = false;
    static double startTimeSeconds = 0.0;

    std::lock_guard<std::mutex> lock(cycleMutex);
    if (!startCaptured) {
        startCaptured = true;
        startTimeSeconds = simulationTimeSeconds;
        logLine("auto-cycle start time captured: " + std::to_string(startTimeSeconds));
    }

    double relative = simulationTimeSeconds - startTimeSeconds;
    if (relative < 0.0) relative = 0.0;
    return relative;
}

[[nodiscard]] VisibleState stateFromRelativeTime(double relativeTimeSeconds) {
    const int index = static_cast<int>(relativeTimeSeconds / kStateSeconds) % 4;
    switch (index) {
    case 0: return VisibleState::Idle;
    case 1: return VisibleState::Walk;
    case 2: return VisibleState::Push;
    case 3: return VisibleState::Climb;
    default: return VisibleState::Idle;
    }
}

[[nodiscard]] double stateLocalTime(double relativeTimeSeconds) {
    double local = std::fmod(relativeTimeSeconds, kStateSeconds);
    if (local < 0.0) local += kStateSeconds;
    return local;
}

void logEvaluationOncePerSecond(
    const char* requestedAnimationCode,
    VisibleState visibleState,
    const arkheon::astsim::AnimationModelOutput& output,
    double simulationTimeSeconds,
    double localTimeSeconds) {
    static std::mutex evalMutex;
    static std::unordered_map<std::string, int> lastLoggedSecondByKey;

    const int currentSecond = static_cast<int>(simulationTimeSeconds);
    const std::string key = std::string(requestedAnimationCode) + "->" + visibleStateName(visibleState);

    {
        std::lock_guard<std::mutex> lock(evalMutex);
        const auto it = lastLoggedSecondByKey.find(key);
        if (it != lastLoggedSecondByKey.end() && it->second == currentSecond) {
            return;
        }
        lastLoggedSecondByKey[key] = currentSecond;
    }

    logLine(
        std::string("evaluate requested=") + requestedAnimationCode +
        " visibleState=" + visibleStateName(visibleState) +
        " t=" + std::to_string(simulationTimeSeconds) +
        " local=" + std::to_string(localTimeSeconds) +
        " jointOverrides=" + std::to_string(output.jointOverrides.size()));
}

[[nodiscard]] bool hasJoint(
    const std::unordered_set<std::string>& availableJointIds,
    const char* jointId) {
    if (!jointId || *jointId == '\0') return false;
    if (availableJointIds.empty()) return true;
    return availableJointIds.find(jointId) != availableJointIds.end();
}

[[nodiscard]] std::unordered_set<std::string> collectJointIds(
    const arkheon::astsim::AnimationModelInput& input) {
    std::unordered_set<std::string> availableJointIds;
    availableJointIds.reserve(input.entity.joints.size());
    for (const auto& joint : input.entity.joints) {
        availableJointIds.insert(joint.jointId);
    }
    return availableJointIds;
}

void logJointIdsOnce(const std::unordered_set<std::string>& availableJointIds) {
    static std::mutex jointLogMutex;
    static bool logged = false;

    std::lock_guard<std::mutex> lock(jointLogMutex);
    if (logged) {
        return;
    }
    logged = true;

    if (availableJointIds.empty()) {
        logLine("joint ids: input.entity.joints was empty; using named overrides only");
        return;
    }

    std::string message = "joint ids available=" + std::to_string(availableJointIds.size()) + ": ";
    int count = 0;
    for (const auto& jointId : availableJointIds) {
        if (count > 0) {
            message += ", ";
        }
        message += jointId;
        ++count;
        if (count >= 80) {
            message += ", ...";
            break;
        }
    }
    logLine(message);
}

void addRawJointDeg(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    const char* jointId,
    double xDeg,
    double yDeg,
    double zDeg) {
    const bool isHip = jointId != nullptr &&
        (std::string(jointId) == "leftHip" || std::string(jointId) == "rightHip");

    // Hard final restraint: no hip writes are allowed to reach the viewer.
    // This catches direct addRawJointDeg calls too, not only addJointDeg.
    if (isHip && kHardBlockHipOverrides) {
        return;
    }

    if (hasJoint(availableJointIds, jointId)) {
        output.jointOverrides.push_back({jointId, deg(xDeg), deg(yDeg), deg(zDeg)});
    }
}

void addJointDeg(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    const char* jointId,
    double xDeg,
    double yDeg,
    double zDeg) {
    const bool isHip = jointId != nullptr &&
        (std::string(jointId) == "leftHip" || std::string(jointId) == "rightHip");

    if (isHip) {
        if (!kAuthorHipOverrides) {
            return;
        }

        // Secondary safety gate before authoring to the viewer. In this build,
        // kHardBlockHipOverrides catches hips in addRawJointDeg too.
        xDeg = clampDeg(xDeg, kHipOutputMinX, kHipOutputMaxX);
        yDeg = clampDeg(yDeg, kHipOutputMinY, kHipOutputMaxY);
        zDeg = clampDeg(zDeg, kHipOutputMinZ, kHipOutputMaxZ);
    }

    addRawJointDeg(availableJointIds, output, jointId, xDeg, yDeg, zDeg);
}

void addLeftHipDeg(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double xDeg,
    double yDeg,
    double zDeg) {
    addJointDeg(availableJointIds, output, "leftHip", xDeg, yDeg, zDeg);
}

void addRightHipDeg(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double xDeg,
    double yDeg,
    double zDeg) {
    addJointDeg(availableJointIds, output, "rightHip", xDeg, yDeg, zDeg);
}

void addPoseDeg(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    const char* jointId,
    const PoseDeg& pose) {
    addJointDeg(availableJointIds, output, jointId, pose.x, pose.y, pose.z);
}

void resetOutput(arkheon::astsim::AnimationModelOutput& output) {
    output.clearExistingJointOverrides = true;
    output.jointOverrides.clear();
}

// -----------------------------------------------------------------------------
// IDLE: exact relaxed upper body, native lower body.
// -----------------------------------------------------------------------------
void addIdleUpperBody(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output) {
    addJointDeg(availableJointIds, output, "leftShoulder", -20.0, 80.0, 0.0);
    addJointDeg(availableJointIds, output, "rightShoulder", -20.0, 80.0, 0.0);
    addJointDeg(availableJointIds, output, "leftElbow", 0.0, 0.0, -15.0);
    addJointDeg(availableJointIds, output, "rightElbow", 0.0, 0.0, -15.0);
}

void addIdleLowerBody(
    const std::unordered_set<std::string>&,
    arkheon::astsim::AnimationModelOutput&) {
    // No lower-body overrides in idle. Preserve native planted stance.
}

// -----------------------------------------------------------------------------
// WALK: 8-phase / 10-joint / XYZ closed-library gait table.
// - The model computes hips, knees, ankles, shoulders, and elbows.
// - The final viewer output remaps the anatomical table to Nathan-safe axes.
// - Hip authoring stays disabled by default to avoid the right-leg pop bug.
// -----------------------------------------------------------------------------
void addWalkLowerBody(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double localTimeSeconds) {
    constexpr double kWalkCycleSeconds = 1.20;
    const double phase = wrap01(localTimeSeconds / kWalkCycleSeconds);
    const WalkPose3D anatomicalPose = computeEightPhaseWalkPose3D(phase);
    const WalkPose3D pose = mapGaitPoseToNathanRig(anatomicalPose);

    // Closed-library output: the model computes all 10 joints, then maps them to Nathan-safe FK.
    addPoseDeg(availableJointIds, output, "leftHip", pose.leftHip);
    addPoseDeg(availableJointIds, output, "rightHip", pose.rightHip);
    addPoseDeg(availableJointIds, output, "leftKnee", pose.leftKnee);
    addPoseDeg(availableJointIds, output, "rightKnee", pose.rightKnee);
    addPoseDeg(availableJointIds, output, "leftAnkle", pose.leftAnkle);
    addPoseDeg(availableJointIds, output, "rightAnkle", pose.rightAnkle);
}

void addWalkUpperBody(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double localTimeSeconds) {
    constexpr double kWalkCycleSeconds = 1.20;
    const double phase = wrap01(localTimeSeconds / kWalkCycleSeconds);
    const WalkPose3D anatomicalPose = computeEightPhaseWalkPose3D(phase);
    const WalkPose3D pose = mapGaitPoseToNathanRig(anatomicalPose);

    // Preserve Nathan shoulder base offset while using gait-driven arm swing.
    addPoseDeg(availableJointIds, output, "leftShoulder", pose.leftShoulder);
    addPoseDeg(availableJointIds, output, "rightShoulder", pose.rightShoulder);
    addPoseDeg(availableJointIds, output, "leftElbow", pose.leftElbow);
    addPoseDeg(availableJointIds, output, "rightElbow", pose.rightElbow);
}

// -----------------------------------------------------------------------------
// PUSH: FK push pose with IK-style braced feet/hands.
// - Lower body braces instead of staying idle.
// - Upper body stays in a push pose and pulses slightly instead of fading to idle.
// -----------------------------------------------------------------------------
void addPushLowerBody(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double localTimeSeconds) {
    constexpr double kPushCycleSeconds = 1.80;
    const double phase = wrap01(localTimeSeconds / kPushCycleSeconds);
    const WalkPose3D actionPose = computePushPose3D(phase);
    const WalkPose3D pose = mapPushPoseToNathanRig(actionPose);

    // The 10-joint model includes hips, but final hard restraints block them for Nathan stability.
    addPoseDeg(availableJointIds, output, "leftHip", pose.leftHip);
    addPoseDeg(availableJointIds, output, "rightHip", pose.rightHip);
    addPoseDeg(availableJointIds, output, "leftKnee", pose.leftKnee);
    addPoseDeg(availableJointIds, output, "rightKnee", pose.rightKnee);
    addPoseDeg(availableJointIds, output, "leftAnkle", pose.leftAnkle);
    addPoseDeg(availableJointIds, output, "rightAnkle", pose.rightAnkle);
}

void addPushUpperBody(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double localTimeSeconds) {
    constexpr double kPushCycleSeconds = 1.80;
    const double phase = wrap01(localTimeSeconds / kPushCycleSeconds);
    const WalkPose3D actionPose = computePushPose3D(phase);
    const WalkPose3D pose = mapPushPoseToNathanRig(actionPose);

    addPoseDeg(availableJointIds, output, "leftShoulder", pose.leftShoulder);
    addPoseDeg(availableJointIds, output, "rightShoulder", pose.rightShoulder);
    addPoseDeg(availableJointIds, output, "leftElbow", pose.leftElbow);
    addPoseDeg(availableJointIds, output, "rightElbow", pose.rightElbow);
}

// -----------------------------------------------------------------------------
// CLIMB: looping alternating FK climb with IK-style hand/foot contact holds.
// - Uses a repeating cycle rather than one non-looping reach over the whole state.
// - Hip cues are computed, but final hip authoring follows kAuthorHipOverrides.
// -----------------------------------------------------------------------------
void addClimbLowerBody(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double localTimeSeconds) {
    constexpr double kClimbCycleSeconds = 2.40;
    const double phase = wrap01(localTimeSeconds / kClimbCycleSeconds);
    const WalkPose3D actionPose = computeClimbPose3D(phase);
    const WalkPose3D pose = mapClimbPoseToNathanRig(actionPose);

    // The 10-joint model includes hips, but final hard restraints block them for Nathan stability.
    addPoseDeg(availableJointIds, output, "leftHip", pose.leftHip);
    addPoseDeg(availableJointIds, output, "rightHip", pose.rightHip);
    addPoseDeg(availableJointIds, output, "leftKnee", pose.leftKnee);
    addPoseDeg(availableJointIds, output, "rightKnee", pose.rightKnee);
    addPoseDeg(availableJointIds, output, "leftAnkle", pose.leftAnkle);
    addPoseDeg(availableJointIds, output, "rightAnkle", pose.rightAnkle);
}

void addClimbUpperBody(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double localTimeSeconds) {
    constexpr double kClimbCycleSeconds = 2.40;
    const double phase = wrap01(localTimeSeconds / kClimbCycleSeconds);
    const WalkPose3D actionPose = computeClimbPose3D(phase);
    const WalkPose3D pose = mapClimbPoseToNathanRig(actionPose);

    addPoseDeg(availableJointIds, output, "leftShoulder", pose.leftShoulder);
    addPoseDeg(availableJointIds, output, "rightShoulder", pose.rightShoulder);
    addPoseDeg(availableJointIds, output, "leftElbow", pose.leftElbow);
    addPoseDeg(availableJointIds, output, "rightElbow", pose.rightElbow);
}

void applyVisibleStatePose(
    VisibleState visibleState,
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double localTimeSeconds) {
    switch (visibleState) {
    case VisibleState::Idle:
        addIdleLowerBody(availableJointIds, output);
        addIdleUpperBody(availableJointIds, output);
        break;
    case VisibleState::Walk:
        addWalkLowerBody(availableJointIds, output, localTimeSeconds);
        addWalkUpperBody(availableJointIds, output, localTimeSeconds);
        break;
    case VisibleState::Push:
        addPushLowerBody(availableJointIds, output, localTimeSeconds);
        addPushUpperBody(availableJointIds, output, localTimeSeconds);
        break;
    case VisibleState::Climb:
        addClimbLowerBody(availableJointIds, output, localTimeSeconds);
        addClimbUpperBody(availableJointIds, output, localTimeSeconds);
        break;
    }
}

[[nodiscard]] bool evaluatePose(
    const char* requestedAnimationCode,
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output) {
    const auto availableJointIds = collectJointIds(input);
    logJointIdsOnce(availableJointIds);
    const double simulationTimeSeconds = input.simulationTimeSeconds;

    VisibleState visibleState = VisibleState::Idle;
    double localTimeSeconds = 0.0;

    if (kAutoCycleDemoMode) {
        const double relativeTimeSeconds = relativeDemoTime(simulationTimeSeconds);
        visibleState = stateFromRelativeTime(relativeTimeSeconds);
        localTimeSeconds = stateLocalTime(relativeTimeSeconds);
    } else {
        visibleState = stateFromRequestedAnimation(requestedAnimationCode);
        // In isolated tuning mode, keep local time running so Walk/Push/Climb
        // loops continue animating while the selected state stays fixed.
        localTimeSeconds = simulationTimeSeconds;
    }

    resetOutput(output);
    applyVisibleStatePose(visibleState, availableJointIds, output, localTimeSeconds);
    logEvaluationOncePerSecond(requestedAnimationCode, visibleState, output, simulationTimeSeconds, localTimeSeconds);

    return !output.jointOverrides.empty();
}

[[nodiscard]] bool evaluateRawanIdleAnimation(
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output) {
    return evaluatePose(kRawanIdleCode, input, output);
}

[[nodiscard]] bool evaluateRawanWalkAnimation(
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output) {
    return evaluatePose(kRawanWalkCode, input, output);
}

[[nodiscard]] bool evaluateRawanPushAnimation(
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output) {
    return evaluatePose(kRawanPushCode, input, output);
}

[[nodiscard]] bool evaluateRawanClimbAnimation(
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output) {
    return evaluatePose(kRawanClimbCode, input, output);
}

arkheon::astsim::IAnimationModel* getNathanPrototype(
    arkheon::astsim::ModelFactoryRegistry* registry) {
    if (!registry) return nullptr;
    auto* prototypeBase = registry->getRegisteredPrototype(kModelType);
    return dynamic_cast<arkheon::astsim::IAnimationModel*>(prototypeBase);
}

} // namespace

int RawanFourMotionPlugin::getInterfaceVersion() const {
    return 1;
}

arkheon::astlib::PluginMetadata RawanFourMotionPlugin::getMetadata() const {
    arkheon::astlib::PluginMetadata metadata;
    metadata.setPluginId("rawan-four-motion-plugin");
    metadata.setVersion("3.6.5-climb-push-action-matrices");
    metadata.setAuthor("Rawan Akrum");
    return metadata;
}

void RawanFourMotionPlugin::initialize(arkheon::astlib::PluginContext& context) {
    logLine("============================================================");
    logLine("initialize: Rawan four motion plugin loaded");
    logLine("version: 3.6.5-climb-push-action-matrices");
    logLine(std::string("mode: ") + animationModeName());
    logLine("design: closed-library 10-joint angle model; separate upper/lower functions per state");
    logLine("FK/IK tune: Walk, Push, and Climb use closed-library 10-joint XYZ FK tables with IK-style stance/contact timing");
    logLine(std::string("hip policy: hard writer-level hip restraints restored; computed internally, authored to viewer: ") + yesNo(kAuthorHipOverrides));
    logLine("walk tune: 8-phase XYZ gait matrix; hips computed internally but hard-blocked; knees/ankles/arms softly clamped; shoulder Y base preserved");
    logLine("push tune: 4-stage heavy-resistance XYZ matrix remapped to Nathan-safe braced knees/ankles/arms; hips hard-blocked at writer");
    logLine("climb tune: 4-stage vertical climb XYZ matrix remapped to Nathan-safe reach/pull poses; hips hard-blocked at writer");
    logLine("tuning note: set kAutoCycleDemoMode=false in RawanFourMotionPlugin.cpp to isolate selected animations");
    logLine("stability note: keep Calib First OFF; right/left hip joints are intentionally not authored because they lift Nathan's leg in the GLB viewer");
    logLine(std::string("direct hip writer enabled: ") + yesNo(kAuthorHipOverrides));
    logLine(std::string("hard hip restraint active: ") + yesNo(kHardBlockHipOverrides));
    logLine("calib note: keep First-frame calibration disabled/OFF; in the latest video ON appears to hold/cancel the bind pose and causes T-pose behavior.");
    logLine(std::string("log path: ") + pluginLogPath());

    initialized_ = true;
    shutdown_ = false;
    rawanIdleRegistered_ = false;
    rawanWalkRegistered_ = false;
    rawanPushRegistered_ = false;
    rawanClimbRegistered_ = false;
    modelType_ = kModelType;

    modelFactoryRegistry_ = nullptr;
    if (context.services) {
        auto* rawService = context.services->getService(arkheon::astsim::IModelPluginService::kPluginServiceId);
        auto* service = static_cast<arkheon::astsim::IModelPluginService*>(rawService);
        modelFactoryRegistry_ = service ? &service->modelFactoryRegistry() : nullptr;
        logLine(std::string("model plugin service found: ") + yesNo(service != nullptr));
    } else {
        logLine("context.services is null");
    }

    auto* prototypeAnimationModel = getNathanPrototype(modelFactoryRegistry_);
    if (!prototypeAnimationModel) {
        logLine(std::string("ERROR: could not find animation prototype: ") + kModelType);
        return;
    }

    logLine(std::string("animation prototype found: ") + kModelType);

    rawanIdleRegistered_ = prototypeAnimationModel->registerAnimation(kRawanIdleCode, evaluateRawanIdleAnimation);
    rawanWalkRegistered_ = prototypeAnimationModel->registerAnimation(kRawanWalkCode, evaluateRawanWalkAnimation);
    rawanPushRegistered_ = prototypeAnimationModel->registerAnimation(kRawanPushCode, evaluateRawanPushAnimation);
    rawanClimbRegistered_ = prototypeAnimationModel->registerAnimation(kRawanClimbCode, evaluateRawanClimbAnimation);

    logLine(std::string("register ") + kRawanIdleCode + ": " + yesNo(rawanIdleRegistered_));
    logLine(std::string("register ") + kRawanWalkCode + ": " + yesNo(rawanWalkRegistered_));
    logLine(std::string("register ") + kRawanPushCode + ": " + yesNo(rawanPushRegistered_));
    logLine(std::string("register ") + kRawanClimbCode + ": " + yesNo(rawanClimbRegistered_));
}

void RawanFourMotionPlugin::tick(double dt) {
    static_cast<void>(dt);

    static bool tickLogged = false;
    if (!tickLogged && initialized_ && !shutdown_) {
        logLine("tick: plugin is active");
        tickLogged = true;
    }

    if (!initialized_ || shutdown_ || !modelFactoryRegistry_) {
        return;
    }
}

void RawanFourMotionPlugin::shutdown() {
    logLine("shutdown: Rawan four motion plugin");

    auto* prototypeAnimationModel = getNathanPrototype(modelFactoryRegistry_);
    if (prototypeAnimationModel) {
        if (rawanIdleRegistered_) {
            static_cast<void>(prototypeAnimationModel->registerAnimation(
                kRawanIdleCode,
                arkheon::astsim::IAnimationModel::AnimationEvaluationFunction {}));
        }
        if (rawanWalkRegistered_) {
            static_cast<void>(prototypeAnimationModel->registerAnimation(
                kRawanWalkCode,
                arkheon::astsim::IAnimationModel::AnimationEvaluationFunction {}));
        }
        if (rawanPushRegistered_) {
            static_cast<void>(prototypeAnimationModel->registerAnimation(
                kRawanPushCode,
                arkheon::astsim::IAnimationModel::AnimationEvaluationFunction {}));
        }
        if (rawanClimbRegistered_) {
            static_cast<void>(prototypeAnimationModel->registerAnimation(
                kRawanClimbCode,
                arkheon::astsim::IAnimationModel::AnimationEvaluationFunction {}));
        }
    }

    rawanIdleRegistered_ = false;
    rawanWalkRegistered_ = false;
    rawanPushRegistered_ = false;
    rawanClimbRegistered_ = false;
    shutdown_ = true;
    modelFactoryRegistry_ = nullptr;
}

} // namespace arkheon::sample::rawanfourmotion

extern "C" {

ARKHEON_ASTLIB_API arkheon::astlib::IPlugin* create_plugin() {
    return new arkheon::sample::rawanfourmotion::RawanFourMotionPlugin();
}

ARKHEON_ASTLIB_API void destroy_plugin(arkheon::astlib::IPlugin* plugin) {
    if (plugin) {
        delete plugin;
    }
}

ARKHEON_ASTLIB_API const char* get_plugin_signature() {
    return "ARKHEON_PLUGIN_V1";
}

} // extern "C"
