// Rawan Character Animation Makeup Project
// Based on the N8RO NathanHuman animation sample plugin structure.

#pragma once

#include <plugin/IPlugin.h>

#include <string>

namespace arkheon::astsim {
class ModelFactoryRegistry;
}

namespace arkheon::sample::rawanfourmotion {

class RawanFourMotionPlugin final : public arkheon::astlib::IPlugin {
public:
    RawanFourMotionPlugin() = default;
    ~RawanFourMotionPlugin() override = default;

    [[nodiscard]] int getInterfaceVersion() const override;
    [[nodiscard]] arkheon::astlib::PluginMetadata getMetadata() const override;

    void initialize(arkheon::astlib::PluginContext& context) override;
    void tick(double dt) override;
    void shutdown() override;

private:
    bool initialized_ = false;
    bool shutdown_ = false;

    bool rawanIdleRegistered_ = false;
    bool rawanWalkRegistered_ = false;
    bool rawanPushRegistered_ = false;
    bool rawanClimbRegistered_ = false;

    std::string modelType_ = "animationModelNathanHuman";
    arkheon::astsim::ModelFactoryRegistry* modelFactoryRegistry_ = nullptr;
};

} // namespace arkheon::sample::rawanfourmotion

extern "C" {
ARKHEON_ASTLIB_API arkheon::astlib::IPlugin* create_plugin();
ARKHEON_ASTLIB_API void destroy_plugin(arkheon::astlib::IPlugin* plugin);
ARKHEON_ASTLIB_API const char* get_plugin_signature();
}
