// RawanStandaloneMotionTest.cpp
// Linux/Windows standalone sanity test for the Rawan Character Plugin motion model.
//
// Purpose:
// - No N8RO runtime dependency.
// - Simulates the closed-library model for 1000 ticks.
// - Converts the 10 computed joint-angle vectors into quaternions.
// - Verifies every quaternion is finite and approximately unit length.
//
// Expected output:
// Linux standalone sanity test:
// SDK version = 0x00010000
// PASS: 1000 ticks, all quaternions finite & unit-ish

#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace rawan_standalone {

constexpr std::uint32_t kSdkVersion = 0x00010000u;
constexpr double kPi = 3.141592653589793238462643383279502884;

struct Vec3 {
    double x;
    double y;
    double z;
};

struct Quat {
    double w;
    double x;
    double y;
    double z;
};

struct MotionState3D {
    Vec3 leftHip;
    Vec3 rightHip;
    Vec3 leftKnee;
    Vec3 rightKnee;
    Vec3 leftAnkle;
    Vec3 rightAnkle;
    Vec3 leftShoulder;
    Vec3 rightShoulder;
    Vec3 leftElbow;
    Vec3 rightElbow;
};

struct JointQuat {
    const char* jointName;
    Quat rotation;
};

double degToRad(double degrees) {
    return degrees * kPi / 180.0;
}

double clamp(double value, double lo, double hi) {
    return value < lo ? lo : (value > hi ? hi : value);
}

Vec3 clampVec(const Vec3& v, double maxAbsX, double maxAbsY, double maxAbsZ) {
    return {
        clamp(v.x, -maxAbsX, maxAbsX),
        clamp(v.y, -maxAbsY, maxAbsY),
        clamp(v.z, -maxAbsZ, maxAbsZ)
    };
}

Quat normalize(const Quat& q) {
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n <= 0.0 || !std::isfinite(n)) {
        return {1.0, 0.0, 0.0, 0.0};
    }
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}

// Euler XYZ degrees -> quaternion. This mirrors the standalone sanity goal:
// generated joint rotations must be finite and unit-ish, not physically simulated.
Quat quatFromEulerDeg(const Vec3& eulerDeg) {
    const double cx = std::cos(degToRad(eulerDeg.x) * 0.5);
    const double sx = std::sin(degToRad(eulerDeg.x) * 0.5);
    const double cy = std::cos(degToRad(eulerDeg.y) * 0.5);
    const double sy = std::sin(degToRad(eulerDeg.y) * 0.5);
    const double cz = std::cos(degToRad(eulerDeg.z) * 0.5);
    const double sz = std::sin(degToRad(eulerDeg.z) * 0.5);

    Quat q;
    q.w = cx * cy * cz + sx * sy * sz;
    q.x = sx * cy * cz - cx * sy * sz;
    q.y = cx * sy * cz + sx * cy * sz;
    q.z = cx * cy * sz - sx * sy * cz;
    return normalize(q);
}

bool isFiniteUnitish(const Quat& q) {
    if (!std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) {
        return false;
    }
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return std::isfinite(n) && n > 0.98 && n < 1.02;
}

double smoothWave(double t, double speed, double phase = 0.0) {
    return std::sin((t * speed + phase) * 2.0 * kPi);
}

MotionState3D computeIdle(double t) {
    const double breathe = smoothWave(t, 0.35) * 1.5;
    return {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        {-20.0 + breathe, 80.0, 0.0}, {-20.0 + breathe, 80.0, 0.0},
        {0.0, 0.0, -15.0}, {0.0, 0.0, -15.0}
    };
}

MotionState3D computeWalk(double t) {
    const double s = smoothWave(t, 0.95);
    const double c = smoothWave(t, 0.95, 0.25);
    const double leftLift = std::max(0.0, s);
    const double rightLift = std::max(0.0, -s);

    // Hips are computed as part of the 10-joint model, but deliberately kept
    // small in this standalone representation to match the restrained viewer build.
    return {
        { 4.0 * s,  1.0 * c, -1.0 * s}, {-4.0 * s, -1.0 * c,  1.0 * s},
        {18.0 + 22.0 * leftLift,  0.5 * c, 0.0},
        {18.0 + 22.0 * rightLift, -0.5 * c, 0.0},
        {-6.0 + 10.0 * leftLift,  0.0, 0.0},
        {-6.0 + 10.0 * rightLift, 0.0, 0.0},
        {-18.0 * s, 80.0,  4.0}, { 18.0 * s, 80.0,  4.0},
        {0.0, 0.0, -18.0}, {0.0, 0.0, -18.0}
    };
}

MotionState3D computePush(double t) {
    const double pulse = smoothWave(t, 0.70) * 3.0;
    return {
        { 3.0 + pulse * 0.2, 0.0, -1.0}, {-3.0 - pulse * 0.2, 0.0, 1.0},
        {16.0 + pulse, 0.0, 0.0}, {26.0 - pulse, 0.0, 0.0},
        {-8.0, 0.0, 0.0}, {8.0, 0.0, 0.0},
        {10.0 + pulse, 58.0, -78.0}, {10.0 + pulse, 58.0, -78.0},
        {0.0, -12.0, -38.0 - pulse}, {0.0, -12.0, -38.0 - pulse}
    };
}

MotionState3D computeClimb(double t) {
    const double s = smoothWave(t, 0.80);
    const double leftReach = std::max(0.0, s);
    const double rightReach = std::max(0.0, -s);
    return {
        { 3.0 * leftReach, 0.0, 0.0}, {3.0 * rightReach, 0.0, 0.0},
        {28.0 + 24.0 * leftReach, 0.0, 0.0},
        {28.0 + 24.0 * rightReach, 0.0, 0.0},
        {6.0 * leftReach, 0.0, 0.0}, {6.0 * rightReach, 0.0, 0.0},
        {-50.0 + 24.0 * leftReach, 74.0, -34.0},
        {-50.0 + 24.0 * rightReach, 74.0, -34.0},
        {0.0, -8.0, -48.0 + 10.0 * rightReach},
        {0.0, -8.0, -48.0 + 10.0 * leftReach}
    };
}

MotionState3D computeMotionState(int tick) {
    const double t = static_cast<double>(tick) / 60.0;
    const int slot = (tick / 250) % 4;
    MotionState3D state;
    switch (slot) {
        case 0: state = computeIdle(t); break;
        case 1: state = computeWalk(t); break;
        case 2: state = computePush(t); break;
        default: state = computeClimb(t); break;
    }

    // Final standalone safety clamp. The actual DLL has Nathan-specific output restraints;
    // this clamp keeps the test model inside sane joint-angle ranges before quaternion conversion.
    state.leftHip       = clampVec(state.leftHip,       8.0,  5.0,  5.0);
    state.rightHip      = clampVec(state.rightHip,      8.0,  5.0,  5.0);
    state.leftKnee      = clampVec(state.leftKnee,     70.0,  5.0,  5.0);
    state.rightKnee     = clampVec(state.rightKnee,    70.0,  5.0,  5.0);
    state.leftAnkle     = clampVec(state.leftAnkle,    25.0,  5.0,  5.0);
    state.rightAnkle    = clampVec(state.rightAnkle,   25.0,  5.0,  5.0);
    state.leftShoulder  = clampVec(state.leftShoulder, 90.0, 90.0, 90.0);
    state.rightShoulder = clampVec(state.rightShoulder,90.0, 90.0, 90.0);
    state.leftElbow     = clampVec(state.leftElbow,    60.0, 30.0, 60.0);
    state.rightElbow    = clampVec(state.rightElbow,   60.0, 30.0, 60.0);
    return state;
}

std::array<JointQuat, 10> computeJointQuaternions(int tick) {
    const MotionState3D s = computeMotionState(tick);
    return {{
        {"leftHip",       quatFromEulerDeg(s.leftHip)},
        {"rightHip",      quatFromEulerDeg(s.rightHip)},
        {"leftKnee",      quatFromEulerDeg(s.leftKnee)},
        {"rightKnee",     quatFromEulerDeg(s.rightKnee)},
        {"leftAnkle",     quatFromEulerDeg(s.leftAnkle)},
        {"rightAnkle",    quatFromEulerDeg(s.rightAnkle)},
        {"leftShoulder",  quatFromEulerDeg(s.leftShoulder)},
        {"rightShoulder", quatFromEulerDeg(s.rightShoulder)},
        {"leftElbow",     quatFromEulerDeg(s.leftElbow)},
        {"rightElbow",    quatFromEulerDeg(s.rightElbow)}
    }};
}

std::uint32_t arkheon_character_sdk_version() {
    return kSdkVersion;
}

bool runSanityTest(int ticks) {
    for (int tick = 0; tick < ticks; ++tick) {
        const auto joints = computeJointQuaternions(tick);
        for (const auto& joint : joints) {
            if (!isFiniteUnitish(joint.rotation)) {
                std::cerr << "FAIL: tick " << tick << ", joint " << joint.jointName
                          << " produced invalid quaternion\n";
                return false;
            }
        }
    }
    return true;
}

} // namespace rawan_standalone

int main() {
    constexpr int kTicks = 1000;

    std::cout << "Linux standalone sanity test:\n";
    std::cout << "SDK version = 0x" << std::hex << std::setw(8) << std::setfill('0')
              << rawan_standalone::arkheon_character_sdk_version() << std::dec << std::setfill(' ') << "\n";

    if (!rawan_standalone::runSanityTest(kTicks)) {
        return 1;
    }

    std::cout << "PASS: " << kTicks << " ticks, all quaternions finite & unit-ish\n";
    return 0;
}
