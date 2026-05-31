// Rawan N8RO visual bridge plugin
// Loads character_plugin_200201852.dll through N8RO's astlib sim-plugin interface
// and registers visual animation evaluators on animationModelNathanHuman.

#pragma once

#include <plugin/IPlugin.h>
#include <cstddef>
#include <cstdint>
#include <string>

#include "arkheon/character/ICharacterController.h"

namespace arkheon::astsim {
class ModelFactoryRegistry;
}

namespace arkheon::sample::simplugin {

class SimPlugin final : public astlib::IPlugin {
public:
    SimPlugin() = default;
    ~SimPlugin() override = default;

    [[nodiscard]] int getInterfaceVersion() const override;
    [[nodiscard]] astlib::PluginMetadata getMetadata() const override;
    void initialize(astlib::PluginContext& context) override;
    void tick(double dt) override;
    void shutdown() override;

private:
    using FnSdkVersion = uint32_t (*)();
    using FnPluginName = const char* (*)();
    using FnGetMotionClips = void (*)(void*, int32_t[3]);
    using FnCreate = void* (*)(const float[10]);
    using FnDestroy = void (*)(void*);
    using FnTick = int32_t (*)(
        void*,
        const arkheon_frame*,
        const arkheon_bone_state[66],
        arkheon_bone_override[10],
        arkheon_vec3*,
        arkheon_quat*,
        const arkheon_input_state*,
        const arkheon_mission_goal*,
        const arkheon_env_api*);

    bool initialized_ = false;
    bool shutdown_ = false;
    double accumulatedDeltaSeconds_ = 0.0;
    std::size_t tickCount_ = 0;
    std::string loadedPluginId_;

    void* characterDll_ = nullptr;
    void* characterHandle_ = nullptr;

    FnSdkVersion characterSdkVersionFn_ = nullptr;
    FnPluginName characterPluginNameFn_ = nullptr;
    FnGetMotionClips characterGetMotionClipsFn_ = nullptr;
    FnCreate characterCreateFn_ = nullptr;
    FnDestroy characterDestroyFn_ = nullptr;
    FnTick characterTickFn_ = nullptr;

    uint32_t characterSdkVersion_ = 0;
    std::string characterPluginName_;
    int32_t motionClips_[3] = { -1, -1, -1 };
    int lastMotionPhase_ = -1;

    arkheon::astsim::ModelFactoryRegistry* modelFactoryRegistry_ = nullptr;
    bool rawanVisualRegistered_ = false;
    std::string modelType_ = "animationModelNathanHuman";

    bool loadCharacterPlugin();
    void unloadCharacterPlugin();
    bool registerRawanVisualAnimations(arkheon::astlib::PluginContext& context);
    void unregisterRawanVisualAnimations();
    void writeLog(const std::string& message) const;
};

} // namespace arkheon::sample::simplugin

extern "C" {
    ARKHEON_ASTLIB_API arkheon::astlib::IPlugin* create_plugin();
    ARKHEON_ASTLIB_API void destroy_plugin(arkheon::astlib::IPlugin* plugin);
    ARKHEON_ASTLIB_API const char* get_plugin_signature();
}
