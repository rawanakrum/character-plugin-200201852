// Rawan Character Animation Makeup Project
// N8RO NathanHuman animation plugin.
//
// Version 3.5.1-stable-presentable-left-hip-only-build-fix
// Purpose of this pass:
// - Stable Calib First OFF workflow.
// - rightHip is never authored because diagnostics showed any rightHip FK override breaks Nathan's native leg.
// - Add only a tiny clamped leftHip offset to reduce stiffness in Walk/Push/Climb.
// - Keep Lua simple; the DLL internally cycles Idle / Walk / Push / Climb.
//
// Axis basis from Rawan's Joint angles2 reference:
// - shoulder/elbow +Y lowers; -Y raises
// - shoulder/elbow -Z moves forward
// - hip/knee/ankle +Z moves leg/foot forward/up
// - elbow X mostly twists, so climb avoids elbow X.

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

// Force the visible four-state demo no matter which Rawan state N8RO keeps requesting.
// This avoids getting stuck when the mission/scenario only evaluates one animation code.
constexpr bool kForceDemoCycleFromAnyRequestedRawanState = true;
constexpr double kAutoCycleSeconds = 5.0;
constexpr double kPi = 3.14159265358979323846;

struct JointPoseDeg {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
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

[[nodiscard]] double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

[[nodiscard]] JointPoseDeg lerpPose(const JointPoseDeg& a, const JointPoseDeg& b, double t) {
    return {lerp(a.x, b.x, t), lerp(a.y, b.y, t), lerp(a.z, b.z, t)};
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


[[nodiscard]] double relativeCycleTime(double simulationTimeSeconds) {
    static std::mutex cycleMutex;
    static bool cycleStartCaptured = false;
    static double cycleStartSeconds = 0.0;

    std::lock_guard<std::mutex> lock(cycleMutex);
    if (!cycleStartCaptured) {
        cycleStartCaptured = true;
        cycleStartSeconds = simulationTimeSeconds;
        logLine(std::string("auto-cycle start time captured: ") + std::to_string(cycleStartSeconds));
    }

    const double relativeTime = simulationTimeSeconds - cycleStartSeconds;
    return relativeTime < 0.0 ? 0.0 : relativeTime;
}

[[nodiscard]] VisibleState visibleStateFromRelativeTime(double relativeTimeSeconds) {
    if (relativeTimeSeconds < 0.0) {
        relativeTimeSeconds = 0.0;
    }

    const int slot = static_cast<int>(relativeTimeSeconds / kAutoCycleSeconds) % 4;
    switch (slot) {
    case 0: return VisibleState::Idle;
    case 1: return VisibleState::Walk;
    case 2: return VisibleState::Push;
    case 3: return VisibleState::Climb;
    default: return VisibleState::Idle;
    }
}

void logEvaluationOncePerSecond(
    const char* requestedAnimationCode,
    VisibleState visibleState,
    const arkheon::astsim::AnimationModelOutput& output,
    double simulationTimeSeconds) {
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
        " jointOverrides=" + std::to_string(output.jointOverrides.size()));
}

[[nodiscard]] bool hasJoint(
    const std::unordered_set<std::string>& availableJointIds,
    const char* jointId) {
    if (!jointId || *jointId == '\0') {
        return false;
    }
    if (availableJointIds.empty()) {
        return true;
    }
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

void addRawJointDeg(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    const char* jointId,
    double xDeg,
    double yDeg,
    double zDeg) {
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
    // Runtime guard: never author hip roots through the normal writer.
    // rightHip is unsafe on this rig; leftHip is allowed only through addSafeLeftHipDeg().
    if (jointId != nullptr &&
        (std::string(jointId) == "rightHip" || std::string(jointId) == "leftHip")) {
        return;
    }

    addRawJointDeg(availableJointIds, output, jointId, xDeg, yDeg, zDeg);
}

void addSafeLeftHipDeg(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double xDeg,
    double yDeg,
    double zDeg) {
    // Controlled leftHip-only offset to reduce stiffness.
    // Conservative clamp: enough to show thigh/weight flow, not enough to cause a flying leg.
    const double x = std::max(-6.0, std::min(6.0, xDeg));
    const double y = std::max(-4.0, std::min(4.0, yDeg));
    const double z = std::max(-16.0, std::min(8.0, zDeg));
    addRawJointDeg(availableJointIds, output, "leftHip", x, y, z);
}

void addPoseDeg(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    const char* jointId,
    const JointPoseDeg& pose) {
    addJointDeg(availableJointIds, output, jointId, pose.x, pose.y, pose.z);
}

void resetOutput(arkheon::astsim::AnimationModelOutput& output) {
    output.clearExistingJointOverrides = true;
    output.jointOverrides.clear();
}

void preserveRightHipNative(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output) {
    // Intentional no-op. Do not output rightHip, not even identity.
    // Skipping lets N8RO/Nathan keep the native planted standing offset.
    (void)availableJointIds;
    (void)output;
}

// -----------------------------------------------------------------------------
// IDLE: exact relaxed upper body, native planted lower body.
// -----------------------------------------------------------------------------
void addIdleUpperBody(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double t) {
    static_cast<void>(t);

    addJointDeg(availableJointIds, output, "leftShoulder", -20.0, 80.0, 0.0);
    addJointDeg(availableJointIds, output, "rightShoulder", -20.0, 80.0, 0.0);
    addJointDeg(availableJointIds, output, "leftElbow", 0.0, 0.0, -15.0);
    addJointDeg(availableJointIds, output, "rightElbow", 0.0, 0.0, -15.0);
}

void addIdleLowerBody(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double t) {
    static_cast<void>(availableJointIds);
    static_cast<void>(output);
    static_cast<void>(t);
    // Intentionally no lower-body overrides.
    // This preserves the native Nathan standing contact and avoids the rightHip float.
}

void applyRawanIdlePose(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double t) {
    addIdleLowerBody(availableJointIds, output, t);
    addIdleUpperBody(availableJointIds, output, t);
}

// -----------------------------------------------------------------------------
// WALK: own upper/lower body. rightHip remains locked/native.
// -----------------------------------------------------------------------------
void addWalkUpperBody(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double t) {
    const double p = std::sin(t * 3.0);
    const double leftArmForward = 0.5 - 0.5 * p;
    const double rightArmForward = 0.5 + 0.5 * p;

    addJointDeg(availableJointIds, output, "leftShoulder", -20.0, 80.0, -14.0 * leftArmForward);
    addJointDeg(availableJointIds, output, "rightShoulder", -20.0, 80.0, -14.0 * rightArmForward);
    addJointDeg(availableJointIds, output, "leftElbow", 0.0, 0.0, -15.0 - 5.0 * leftArmForward);
    addJointDeg(availableJointIds, output, "rightElbow", 0.0, 0.0, -15.0 - 5.0 * rightArmForward);
}

void addWalkLowerBody(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double localTimeSeconds) {
    const double p = std::sin(localTimeSeconds * 2.6);
    const double leftStep = 0.5 + 0.5 * p;
    const double rightSupport = 1.0 - leftStep;

    // Tiny leftHip micro-swing gives the walk some thigh/weight flow.
    // rightHip stays native; do not author it.
    addSafeLeftHipDeg(availableJointIds, output, 0.0, 1.5 * p, -6.0 + 8.0 * p);
    addJointDeg(availableJointIds, output, "leftKnee", 0.0, 0.0, -10.0 - 24.0 * leftStep);
    addJointDeg(availableJointIds, output, "leftAnkle", 0.0, 0.0, -2.0 + 10.0 * leftStep);

    preserveRightHipNative(availableJointIds, output);
    addJointDeg(availableJointIds, output, "rightKnee", 0.0, 0.0, -8.0 - 22.0 * rightSupport);
    addJointDeg(availableJointIds, output, "rightAnkle", 0.0, 0.0, -4.0 + 8.0 * rightSupport);
}


void applyRawanWalkPose(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double t) {
    addWalkLowerBody(availableJointIds, output, t);
    addWalkUpperBody(availableJointIds, output, t);
}

// -----------------------------------------------------------------------------
// PUSH: own upper/lower body. Lower body is mostly native/planted.
// -----------------------------------------------------------------------------
void addPushUpperBody(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double t) {
    const double pulse = 0.75 + 0.25 * std::sin(t * 2.1);
    const JointPoseDeg shoulderIdle {-20.0, 80.0, 0.0};
    const JointPoseDeg shoulderPush {4.0, 40.0, -88.0};
    const JointPoseDeg elbowIdle {0.0, 0.0, -15.0};
    const JointPoseDeg elbowPush {0.0, 2.0, -55.0};

    addPoseDeg(availableJointIds, output, "leftShoulder", lerpPose(shoulderIdle, shoulderPush, pulse));
    addPoseDeg(availableJointIds, output, "rightShoulder", lerpPose(shoulderIdle, shoulderPush, pulse));
    addPoseDeg(availableJointIds, output, "leftElbow", lerpPose(elbowIdle, elbowPush, pulse));
    addPoseDeg(availableJointIds, output, "rightElbow", lerpPose(elbowIdle, elbowPush, pulse));
}

void addPushLowerBody(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double localTimeSeconds) {
    const double brace = smooth01(0.5 - 0.5 * std::cos(localTimeSeconds * 2.2));

    // Small braced leftHip only; rightHip remains native/skipped.
    addSafeLeftHipDeg(availableJointIds, output, 0.0, -1.0 * brace, -8.0 - 3.0 * brace);
    preserveRightHipNative(availableJointIds, output);
    addJointDeg(availableJointIds, output, "leftKnee", 0.0, 0.0, -12.0 - 10.0 * brace);
    addJointDeg(availableJointIds, output, "rightKnee", 0.0, 0.0, -12.0 - 10.0 * brace);
    addJointDeg(availableJointIds, output, "leftAnkle", 0.0, 0.0, -4.0 + 3.0 * brace);
    addJointDeg(availableJointIds, output, "rightAnkle", 0.0, 0.0, -4.0 + 3.0 * brace);
}


void applyRawanPushPose(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double t) {
    addPushLowerBody(availableJointIds, output, t);
    addPushUpperBody(availableJointIds, output, t);
}

// -----------------------------------------------------------------------------
// CLIMB: own upper/lower body. rightHip remains locked/native.
// -----------------------------------------------------------------------------
[[nodiscard]] double climbReachAmount(double t) {
    // 2.2s mini-cycle inside the 5s Rawan Climb segment:
    // reach upward, hold/pull, then reset. This avoids the old continuous sine wave
    // that passed through ugly halfway poses.
    const double phase = std::fmod(t, 2.2) / 2.2;
    if (phase < 0.24) {
        return smooth01(phase / 0.24);
    }
    if (phase < 0.74) {
        return 1.0;
    }
    return 1.0 - smooth01((phase - 0.74) / 0.26);
}

[[nodiscard]] double climbPullAmount(double t) {
    // Pull happens after the reach is established: the high arm bends and the lifted
    // leg settles slightly, like pulling onto the next rung.
    const double phase = std::fmod(t, 2.2) / 2.2;
    if (phase < 0.40 || phase > 0.86) {
        return 0.0;
    }
    if (phase < 0.62) {
        return smooth01((phase - 0.40) / 0.22);
    }
    return 1.0 - smooth01((phase - 0.62) / 0.24);
}

void addClimbUpperBody(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double t) {
    const double reach = climbReachAmount(t);
    const double pull = climbPullAmount(t);

    // Ladder-style silhouette:
    // right arm = higher reaching/pulling arm
    // left arm  = lower gripping arm
    // Avoid huge elbow/shoulder X twist; PowerPoint says X mostly twists, while
    // -Y raises the arm/forearm and -Z moves the arm/forearm forward.
    const JointPoseDeg rightShoulderPrepare {-20.0, 18.0, -42.0};
    const JointPoseDeg rightShoulderReach   {-14.0, -28.0, -92.0};
    const JointPoseDeg rightShoulderPull    {-18.0, -12.0, -82.0};

    const JointPoseDeg leftShoulderPrepare  {-20.0, 42.0, -20.0};
    const JointPoseDeg leftShoulderGrip     {18.0,  -6.0, -72.0};
    const JointPoseDeg leftShoulderPull     {24.0,   4.0, -66.0};

    const JointPoseDeg rightElbowPrepare {0.0,  0.0, -15.0};
    const JointPoseDeg rightElbowReach   {0.0, -34.0, -24.0};
    const JointPoseDeg rightElbowPull    {0.0, -66.0, -18.0};

    const JointPoseDeg leftElbowPrepare  {0.0,   0.0, -15.0};
    const JointPoseDeg leftElbowGrip     {0.0, -54.0, -20.0};
    const JointPoseDeg leftElbowPull     {0.0, -70.0, -16.0};

    const JointPoseDeg rightShoulder = lerpPose(
        lerpPose(rightShoulderPrepare, rightShoulderReach, reach),
        rightShoulderPull,
        pull);
    const JointPoseDeg leftShoulder = lerpPose(
        lerpPose(leftShoulderPrepare, leftShoulderGrip, reach),
        leftShoulderPull,
        pull);
    const JointPoseDeg rightElbow = lerpPose(
        lerpPose(rightElbowPrepare, rightElbowReach, reach),
        rightElbowPull,
        pull);
    const JointPoseDeg leftElbow = lerpPose(
        lerpPose(leftElbowPrepare, leftElbowGrip, reach),
        leftElbowPull,
        pull);

    addPoseDeg(availableJointIds, output, "rightShoulder", rightShoulder);
    addPoseDeg(availableJointIds, output, "leftShoulder", leftShoulder);
    addPoseDeg(availableJointIds, output, "rightElbow", rightElbow);
    addPoseDeg(availableJointIds, output, "leftElbow", leftElbow);
}

void addClimbLowerBody(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double t) {
    const double reach = climbReachAmount(t);
    const double pull = climbPullAmount(t);

    // Left leg performs the visible knee-drive. rightHip remains native/locked.
    // Keep hip +Z high enough to read as climbing, but not as extreme as the old 70deg.
    addSafeLeftHipDeg(availableJointIds, output, 0.0, 0.0, 0.20 * (lerp(8.0, 58.0, reach) - 8.0 * pull) - 8.0);
    addJointDeg(availableJointIds, output, "leftKnee", 0.0, 0.0, lerp(-8.0, -64.0, reach) + 6.0 * pull);
    addJointDeg(availableJointIds, output, "leftAnkle", 0.0, 0.0, lerp(0.0, 24.0, reach));

    // Supporting right leg: no rightHip override. Only a small knee/ankle brace so it
    // participates without flying upward.
    addJointDeg(availableJointIds, output, "rightKnee", 0.0, 0.0, lerp(-3.0, -10.0, reach));
    addJointDeg(availableJointIds, output, "rightAnkle", 0.0, 0.0, lerp(-4.0, -8.0, reach));
}

void applyRawanClimbPose(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double t) {
    addClimbLowerBody(availableJointIds, output, t);
    addClimbUpperBody(availableJointIds, output, t);
}

void applyVisibleStatePose(
    VisibleState visibleState,
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    double t) {
    switch (visibleState) {
    case VisibleState::Idle:
        applyRawanIdlePose(availableJointIds, output, t);
        break;
    case VisibleState::Walk:
        applyRawanWalkPose(availableJointIds, output, t);
        break;
    case VisibleState::Push:
        applyRawanPushPose(availableJointIds, output, t);
        break;
    case VisibleState::Climb:
        applyRawanClimbPose(availableJointIds, output, t);
        break;
    }
}

[[nodiscard]] bool evaluateCommonAnimation(
    const char* requestedAnimationCode,
    VisibleState requestedState,
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output) {
    const auto availableJointIds = collectJointIds(input);
    const double t = input.simulationTimeSeconds;

    resetOutput(output);

    const double relativeTime = relativeCycleTime(t);
    double stateLocalTime = t;

    VisibleState visibleState = requestedState;
    if (kForceDemoCycleFromAnyRequestedRawanState) {
        visibleState = visibleStateFromRelativeTime(relativeTime);
        stateLocalTime = std::fmod(relativeTime, kAutoCycleSeconds);
    }

    applyVisibleStatePose(visibleState, availableJointIds, output, stateLocalTime);
    logEvaluationOncePerSecond(requestedAnimationCode, visibleState, output, t);

    return !output.jointOverrides.empty();
}

[[nodiscard]] bool evaluateRawanIdleAnimation(
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output) {
    return evaluateCommonAnimation(kRawanIdleCode, VisibleState::Idle, input, output);
}

[[nodiscard]] bool evaluateRawanWalkAnimation(
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output) {
    return evaluateCommonAnimation(kRawanWalkCode, VisibleState::Walk, input, output);
}

[[nodiscard]] bool evaluateRawanPushAnimation(
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output) {
    return evaluateCommonAnimation(kRawanPushCode, VisibleState::Push, input, output);
}

[[nodiscard]] bool evaluateRawanClimbAnimation(
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output) {
    return evaluateCommonAnimation(kRawanClimbCode, VisibleState::Climb, input, output);
}

arkheon::astsim::IAnimationModel* getNathanPrototype(
    arkheon::astsim::ModelFactoryRegistry* registry) {
    if (!registry) {
        return nullptr;
    }
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
    metadata.setVersion("3.5.1-stable-presentable-left-hip-only-build-fix");
    metadata.setAuthor("Rawan Akrum");
    return metadata;
}

void RawanFourMotionPlugin::initialize(arkheon::astlib::PluginContext& context) {
    initialized_ = true;
    shutdown_ = false;

    rawanIdleRegistered_ = false;
    rawanWalkRegistered_ = false;
    rawanPushRegistered_ = false;
    rawanClimbRegistered_ = false;
    modelType_ = kModelType;

    logLine("============================================================");
    logLine("initialize: Rawan four motion plugin loaded");
    logLine("version: 3.5.1-stable-presentable-left-hip-only-build-fix");
    logLine(std::string("log path: ") + pluginLogPath());
    logLine("design: separate upper/lower functions for every state; no shared idle/breathing helper");
    logLine("test setting: Calib First OFF");
    logLine("hip policy: rightHip always skipped/native; tiny clamped leftHip only for Walk/Push/Climb");
    logLine("runtime guard: normal addJointDeg skips BOTH hips; addSafeLeftHipDeg is the only hip writer");
    logLine("motion tune: final stable presentable pass; less stiff non-rightHip Walk/Push/Climb");
    logLine(std::string("force demo cycle from any requested Rawan state: ") + yesNo(kForceDemoCycleFromAnyRequestedRawanState));

    modelFactoryRegistry_ = nullptr;
    if (context.services) {
        auto* rawService = context.services->getService(arkheon::astsim::IModelPluginService::kPluginServiceId);
        auto* service = static_cast<arkheon::astsim::IModelPluginService*>(rawService);
        modelFactoryRegistry_ = service ? &service->modelFactoryRegistry() : nullptr;
        logLine(std::string("model plugin service found: ") + yesNo(service != nullptr));
    } else {
        logLine("model plugin service found: no");
    }

    auto* prototypeAnimationModel = getNathanPrototype(modelFactoryRegistry_);
    logLine(std::string("animation prototype found: ") + yesNo(prototypeAnimationModel != nullptr));

    if (!prototypeAnimationModel) {
        return;
    }

    rawanIdleRegistered_ = prototypeAnimationModel->registerAnimation(
        kRawanIdleCode,
        evaluateRawanIdleAnimation);
    rawanWalkRegistered_ = prototypeAnimationModel->registerAnimation(
        kRawanWalkCode,
        evaluateRawanWalkAnimation);
    rawanPushRegistered_ = prototypeAnimationModel->registerAnimation(
        kRawanPushCode,
        evaluateRawanPushAnimation);
    rawanClimbRegistered_ = prototypeAnimationModel->registerAnimation(
        kRawanClimbCode,
        evaluateRawanClimbAnimation);

    logLine(std::string("register Rawan Idle: ") + yesNo(rawanIdleRegistered_));
    logLine(std::string("register Rawan Walk: ") + yesNo(rawanWalkRegistered_));
    logLine(std::string("register Rawan Push: ") + yesNo(rawanPushRegistered_));
    logLine(std::string("register Rawan Climb: ") + yesNo(rawanClimbRegistered_));
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
