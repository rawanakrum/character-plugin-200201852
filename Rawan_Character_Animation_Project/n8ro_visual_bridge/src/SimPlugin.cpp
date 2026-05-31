// Rawan N8RO visual bridge plugin
// 1) Loads character_plugin_200201852.dll
// 2) Calls arkheon_character_tick for verification/logging
// 3) Registers Rawan-powered visual states on Nathan's animation model:
//      Idle Shake      -> PUSH-LIKE joint overrides
//      Idle Breathing  -> CLIMB-LIKE joint overrides
//    Keep Idle Walk Forward as N8RO's real walking clip.

#include "SimPlugin.h"

#include <ISimulationEngine.h>
#include <model/AnimationModel.h>
#include <model/ModelFactoryRegistry.h>
#include <plugin/IModelPluginService.h>
#include <plugin/PluginContext.h>
#include <plugin/IPluginServices.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

namespace fs = std::filesystem;

namespace arkheon::sample::simplugin {

namespace {

// Static bridge state used by the animation evaluator callbacks registered in N8RO.
// N8RO's sample API registers free functions, so this is intentionally process-local.
using RawanTickFn = int32_t (*)(
    void*,
    const arkheon_frame*,
    const arkheon_bone_state[66],
    arkheon_bone_override[10],
    arkheon_vec3*,
    arkheon_quat*,
    const arkheon_input_state*,
    const arkheon_mission_goal*,
    const arkheon_env_api*);

struct RawanRuntimeState {
    void* handle = nullptr;
    RawanTickFn tick = nullptr;
    std::uint64_t frameNumber = 0;
    double lastTimeByMode[3] = { 0.0, 0.0, 0.0 };
};

RawanRuntimeState g_rawan;

arkheon_quat identityQuat() {
    arkheon_quat q{ 0.0f, 0.0f, 0.0f, 1.0f };
    return q;
}

std::string getModuleDirectory() {
    HMODULE module = nullptr;
    if (!GetModuleHandleExA(
            GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            reinterpret_cast<LPCSTR>(&getModuleDirectory),
            &module)) {
        return fs::current_path().string();
    }

    char buffer[MAX_PATH] = {};
    DWORD len = GetModuleFileNameA(module, buffer, MAX_PATH);
    if (len == 0 || len >= MAX_PATH) {
        return fs::current_path().string();
    }

    return fs::path(buffer).parent_path().string();
}

std::string toHex(uint32_t value) {
    char buffer[32] = {};
    std::snprintf(buffer, sizeof(buffer), "0x%08X", value);
    return std::string(buffer);
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

struct EulerXYZ {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

[[nodiscard]] EulerXYZ quatToEulerXYZ(const arkheon_quat& qIn) {
    // Standard quaternion -> Euler conversion. N8RO sample animation models use radians.
    double x = static_cast<double>(qIn.x);
    double y = static_cast<double>(qIn.y);
    double z = static_cast<double>(qIn.z);
    double w = static_cast<double>(qIn.w);

    const double norm = std::sqrt((x * x) + (y * y) + (z * z) + (w * w));
    if (norm > 1.0e-9) {
        x /= norm;
        y /= norm;
        z /= norm;
        w /= norm;
    }

    EulerXYZ out;

    const double sinr_cosp = 2.0 * ((w * x) + (y * z));
    const double cosr_cosp = 1.0 - (2.0 * ((x * x) + (y * y)));
    out.x = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * ((w * y) - (z * x));
    if (std::abs(sinp) >= 1.0) {
        out.y = std::copysign(1.57079632679489661923, sinp);
    } else {
        out.y = std::asin(sinp);
    }

    const double siny_cosp = 2.0 * ((w * z) + (x * y));
    const double cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)));
    out.z = std::atan2(siny_cosp, cosy_cosp);

    return out;
}

void pushJointIfPresent(
    arkheon::astsim::AnimationModelOutput& output,
    const std::unordered_set<std::string>& availableJointIds,
    const char* jointId,
    const arkheon_bone_override& overrideValue) {
    if (!overrideValue.apply || !hasJoint(availableJointIds, jointId)) {
        return;
    }

    const auto e = quatToEulerXYZ(overrideValue.local_rotation);
    output.jointOverrides.push_back({ jointId, e.x, e.y, e.z });
}

[[nodiscard]] bool evaluateRawanMode(
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output,
    int mode) {
    if (!g_rawan.handle || !g_rawan.tick) {
        return false;
    }

    std::unordered_set<std::string> availableJointIds;
    availableJointIds.reserve(input.entity.joints.size());
    for (const auto& joint : input.entity.joints) {
        availableJointIds.insert(joint.jointId);
    }

    const double t = input.simulationTimeSeconds;
    double dt = 0.02;
    if (mode >= 0 && mode < 3) {
        const double previous = g_rawan.lastTimeByMode[mode];
        if (previous > 0.0 && t > previous) {
            dt = std::clamp(t - previous, 0.001, 0.05);
        }
        g_rawan.lastTimeByMode[mode] = t;
    }

    arkheon_frame frame{};
    frame.simulation_time_s = t;
    frame.delta_time_s = dt;
    frame.frame_number = ++g_rawan.frameNumber;
    frame.is_paused = 0;

    arkheon_bone_state bones[66]{};
    for (auto& bone : bones) {
        bone.local_rotation = identityQuat();
    }

    arkheon_bone_override overrides[ARK_JOINT_COUNT]{};
    arkheon_vec3 rootDelta{};
    arkheon_quat rootRotationDelta = identityQuat();
    arkheon_input_state inputState{};

    if (mode == 0) {
        inputState.hotkey_motion_a = 1;
    } else if (mode == 1) {
        inputState.hotkey_motion_b = 1;
    } else {
        inputState.hotkey_motion_c = 1;
    }

    const int32_t rc = g_rawan.tick(
        g_rawan.handle,
        &frame,
        bones,
        overrides,
        &rootDelta,
        &rootRotationDelta,
        &inputState,
        nullptr,
        nullptr);

    if (rc != 0) {
        return false;
    }

    output.clearExistingJointOverrides = true;
    output.jointOverrides.clear();

    // Map the 10-joint course ABI to N8RO's configured Nathan joint IDs.
    pushJointIfPresent(output, availableJointIds, "leftShoulder",  overrides[ARK_JOINT_UPPERARM_L]);
    pushJointIfPresent(output, availableJointIds, "rightShoulder", overrides[ARK_JOINT_UPPERARM_R]);
    pushJointIfPresent(output, availableJointIds, "leftElbow",     overrides[ARK_JOINT_LOWERARM_L]);
    pushJointIfPresent(output, availableJointIds, "rightElbow",    overrides[ARK_JOINT_LOWERARM_R]);
    pushJointIfPresent(output, availableJointIds, "leftHip",       overrides[ARK_JOINT_THIGH_L]);
    pushJointIfPresent(output, availableJointIds, "rightHip",      overrides[ARK_JOINT_THIGH_R]);
    pushJointIfPresent(output, availableJointIds, "leftKnee",      overrides[ARK_JOINT_CALF_L]);
    pushJointIfPresent(output, availableJointIds, "rightKnee",     overrides[ARK_JOINT_CALF_R]);
    pushJointIfPresent(output, availableJointIds, "leftAnkle",     overrides[ARK_JOINT_FOOT_L]);
    pushJointIfPresent(output, availableJointIds, "rightAnkle",    overrides[ARK_JOINT_FOOT_R]);

    return !output.jointOverrides.empty();
}

[[nodiscard]] bool evaluateRawanPush(
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output) {
    return evaluateRawanMode(input, output, 1);
}

[[nodiscard]] bool evaluateRawanClimb(
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output) {
    return evaluateRawanMode(input, output, 2);
}

} // namespace

int SimPlugin::getInterfaceVersion() const {
    return 1;
}

astlib::PluginMetadata SimPlugin::getMetadata() const {
    astlib::PluginMetadata metadata;
    metadata.setPluginId("rawan-character-bridge");
    metadata.setVersion("1.1.0");
    metadata.setAuthor("Rawan Character Animation Project");
    return metadata;
}

void SimPlugin::initialize(astlib::PluginContext& context) {
    initialized_ = true;
    shutdown_ = false;
    accumulatedDeltaSeconds_ = 0.0;
    tickCount_ = 0;
    lastMotionPhase_ = -1;
    rawanVisualRegistered_ = false;

    loadedPluginId_ = context.metadata.pluginId();
    if (loadedPluginId_.empty()) {
        loadedPluginId_ = "rawan-character-bridge";
    }

    writeLog("initialize: visual bridge loaded by N8RO as pluginId=" + loadedPluginId_);

    if (loadCharacterPlugin()) {
        std::ostringstream os;
        os << "character DLL loaded: name='" << characterPluginName_
           << "' sdk=" << toHex(characterSdkVersion_)
           << " clips=[" << motionClips_[0] << ", " << motionClips_[1] << ", " << motionClips_[2] << "]";
        writeLog(os.str());
    } else {
        writeLog("ERROR: character DLL failed to load. Check rawan_bridge.log path and payload placement.");
    }

    if (registerRawanVisualAnimations(context)) {
        writeLog("visual registration ok: Idle Shake -> PUSH-LIKE, Idle Breathing -> CLIMB-LIKE");
    } else {
        writeLog("WARNING: visual registration failed. DLL still ticks, but GLB viewer may keep built-in clips.");
    }
}

bool SimPlugin::registerRawanVisualAnimations(arkheon::astlib::PluginContext& context) {
    modelFactoryRegistry_ = nullptr;

    if (context.services) {
        auto* rawService = context.services->getService(arkheon::astsim::IModelPluginService::kPluginServiceId);
        auto* service = static_cast<arkheon::astsim::IModelPluginService*>(rawService);
        modelFactoryRegistry_ = service ? &service->modelFactoryRegistry() : nullptr;
    }

    if (!modelFactoryRegistry_) {
        writeLog("WARNING: IModelPluginService not available.");
        return false;
    }

    auto* prototypeBase = modelFactoryRegistry_->getRegisteredPrototype(modelType_);
    auto* prototypeAnimationModel = dynamic_cast<arkheon::astsim::IAnimationModel*>(prototypeBase);
    if (!prototypeAnimationModel) {
        writeLog("WARNING: animationModelNathanHuman prototype not found or not IAnimationModel.");
        return false;
    }

    const bool pushOk = prototypeAnimationModel->registerAnimation("Idle Shake", evaluateRawanPush);
    const bool climbOk = prototypeAnimationModel->registerAnimation("Idle Breathing", evaluateRawanClimb);
    rawanVisualRegistered_ = pushOk && climbOk;
    return rawanVisualRegistered_;
}

void SimPlugin::unregisterRawanVisualAnimations() {
    if (!modelFactoryRegistry_ || !rawanVisualRegistered_) {
        return;
    }

    auto* prototypeBase = modelFactoryRegistry_->getRegisteredPrototype(modelType_);
    auto* prototypeAnimationModel = dynamic_cast<arkheon::astsim::IAnimationModel*>(prototypeBase);
    if (prototypeAnimationModel) {
        static_cast<void>(prototypeAnimationModel->registerAnimation(
            "Idle Shake",
            arkheon::astsim::IAnimationModel::AnimationEvaluationFunction {}));
        static_cast<void>(prototypeAnimationModel->registerAnimation(
            "Idle Breathing",
            arkheon::astsim::IAnimationModel::AnimationEvaluationFunction {}));
    }

    rawanVisualRegistered_ = false;
}

bool SimPlugin::loadCharacterPlugin() {
    unloadCharacterPlugin();

    const fs::path selfDir = getModuleDirectory();
    const std::array<fs::path, 4> candidates = {
        selfDir / "rawan" / "character_plugin_200201852.dll",
        selfDir / "character_plugin_200201852.dll",
        fs::current_path() / "character_plugin_200201852.dll",
        fs::current_path() / "rawan" / "character_plugin_200201852.dll"
    };

    HMODULE dll = nullptr;
    fs::path chosenPath;
    for (const auto& candidate : candidates) {
        if (!fs::exists(candidate)) {
            continue;
        }
        dll = LoadLibraryA(candidate.string().c_str());
        if (dll) {
            chosenPath = candidate;
            break;
        }
    }

    if (!dll) {
        writeLog("ERROR: LoadLibrary failed for character_plugin_200201852.dll. Expected it under userPlugins\\sim\\rawan\\");
        return false;
    }

    characterDll_ = dll;

    characterSdkVersionFn_ = reinterpret_cast<FnSdkVersion>(GetProcAddress(dll, "arkheon_character_sdk_version"));
    characterPluginNameFn_ = reinterpret_cast<FnPluginName>(GetProcAddress(dll, "arkheon_character_plugin_name"));
    characterGetMotionClipsFn_ = reinterpret_cast<FnGetMotionClips>(GetProcAddress(dll, "arkheon_character_get_motion_clips"));
    characterCreateFn_ = reinterpret_cast<FnCreate>(GetProcAddress(dll, "arkheon_character_create"));
    characterDestroyFn_ = reinterpret_cast<FnDestroy>(GetProcAddress(dll, "arkheon_character_destroy"));
    characterTickFn_ = reinterpret_cast<FnTick>(GetProcAddress(dll, "arkheon_character_tick"));

    if (!characterSdkVersionFn_ || !characterPluginNameFn_ || !characterGetMotionClipsFn_ ||
        !characterCreateFn_ || !characterDestroyFn_ || !characterTickFn_) {
        writeLog("ERROR: one or more arkheon_character_* exports are missing from the DLL.");
        unloadCharacterPlugin();
        return false;
    }

    characterSdkVersion_ = characterSdkVersionFn_();
    const char* name = characterPluginNameFn_();
    characterPluginName_ = name ? name : "<null>";

    const float segmentLengths[10] = {
        0.30f, 0.30f, 0.25f, 0.25f, 0.42f,
        0.42f, 0.40f, 0.40f, 0.12f, 0.12f
    };

    characterHandle_ = characterCreateFn_(segmentLengths);
    if (!characterHandle_) {
        writeLog("ERROR: arkheon_character_create returned null.");
        unloadCharacterPlugin();
        return false;
    }

    characterGetMotionClipsFn_(characterHandle_, motionClips_);
    writeLog("LoadLibrary path: " + chosenPath.string());

    g_rawan.handle = characterHandle_;
    g_rawan.tick = characterTickFn_;
    g_rawan.frameNumber = 0;
    g_rawan.lastTimeByMode[0] = 0.0;
    g_rawan.lastTimeByMode[1] = 0.0;
    g_rawan.lastTimeByMode[2] = 0.0;

    return true;
}

void SimPlugin::tick(double dt) {
    if (!initialized_ || shutdown_) {
        return;
    }

    const auto sampleState = astsim::SimulationState::Idle;
    (void)sampleState;

    if (dt > 0.0) {
        accumulatedDeltaSeconds_ += dt;
    }
    ++tickCount_;

    if (!characterHandle_ || !characterTickFn_) {
        return;
    }

    arkheon_frame frame{};
    frame.simulation_time_s = accumulatedDeltaSeconds_;
    frame.delta_time_s = dt > 0.0 ? dt : 0.02;
    frame.frame_number = static_cast<uint64_t>(tickCount_);
    frame.is_paused = 0;

    arkheon_bone_state bones[66]{};
    for (auto& bone : bones) {
        bone.local_rotation = identityQuat();
    }

    arkheon_bone_override overrides[ARK_JOINT_COUNT]{};
    arkheon_vec3 rootDelta{};
    arkheon_quat rootRotationDelta = identityQuat();
    arkheon_input_state input{};

    const int phase = static_cast<int>(accumulatedDeltaSeconds_ / 3.0) % 3;
    if (phase != lastMotionPhase_) {
        lastMotionPhase_ = phase;
        if (phase == 0) {
            input.hotkey_motion_a = 1;
            writeLog("state -> WALK-LIKE");
        } else if (phase == 1) {
            input.hotkey_motion_b = 1;
            writeLog("state -> PUSH-LIKE");
        } else {
            input.hotkey_motion_c = 1;
            writeLog("state -> CLIMB-LIKE");
        }
    }

    const int32_t rc = characterTickFn_(
        characterHandle_,
        &frame,
        bones,
        overrides,
        &rootDelta,
        &rootRotationDelta,
        &input,
        nullptr,
        nullptr);

    if (rc != 0) {
        writeLog("ERROR: arkheon_character_tick returned non-zero");
        return;
    }

    if (tickCount_ % 150 == 0) {
        std::ostringstream os;
        os << "tick ok frame=" << tickCount_
           << " rootDelta=(" << rootDelta.x << ", " << rootDelta.y << ", " << rootDelta.z << ")"
           << " firstJointApply=" << static_cast<int>(overrides[0].apply)
           << " visualRegistered=" << (rawanVisualRegistered_ ? 1 : 0);
        writeLog(os.str());
    }
}

void SimPlugin::shutdown() {
    writeLog("shutdown: bridge unloading");
    shutdown_ = true;
    unregisterRawanVisualAnimations();
    unloadCharacterPlugin();
    modelFactoryRegistry_ = nullptr;
}

void SimPlugin::unloadCharacterPlugin() {
    g_rawan.handle = nullptr;
    g_rawan.tick = nullptr;

    if (characterHandle_ && characterDestroyFn_) {
        characterDestroyFn_(characterHandle_);
    }
    characterHandle_ = nullptr;

    if (characterDll_) {
        FreeLibrary(static_cast<HMODULE>(characterDll_));
    }
    characterDll_ = nullptr;

    characterSdkVersionFn_ = nullptr;
    characterPluginNameFn_ = nullptr;
    characterGetMotionClipsFn_ = nullptr;
    characterCreateFn_ = nullptr;
    characterDestroyFn_ = nullptr;
    characterTickFn_ = nullptr;

    characterSdkVersion_ = 0;
    characterPluginName_.clear();
    motionClips_[0] = motionClips_[1] = motionClips_[2] = -1;
}

void SimPlugin::writeLog(const std::string& message) const {
    const fs::path logPath = fs::path(getModuleDirectory()) / "rawan_bridge.log";
    std::ofstream out(logPath, std::ios::app);
    if (out) {
        out << message << '\n';
    }
    OutputDebugStringA(("[rawan-character-bridge] " + message + "\n").c_str());
}

} // namespace arkheon::sample::simplugin

extern "C" {

ARKHEON_ASTLIB_API arkheon::astlib::IPlugin* create_plugin() {
    return new arkheon::sample::simplugin::SimPlugin();
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
