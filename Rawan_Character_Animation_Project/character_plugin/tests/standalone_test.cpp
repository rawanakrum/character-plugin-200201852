// Standalone harness — NO APP required.
// Build: cl /std:c++17 /EHsc tests\standalone_test.cpp src\StudentController.cpp /I include
// Run:   .\standalone_test.exe
#include "arkheon/character/ICharacterController.h"
#include <cstdio>
#include <cmath>

/* Mock env_api — returns constants so your plugin can run offline. */
static int32_t mock_raycast(void*, arkheon_vec3, arkheon_vec3, float,
                            arkheon_vec3* h, arkheon_vec3* n, int32_t* id) {
    *h = {0,0,0}; *n = {0,1,0}; *id = -1; return 0;
}
static int32_t mock_aabb(void*, int32_t, arkheon_vec3* mn, arkheon_vec3* mx) {
    *mn = {-1,-1,-1}; *mx = {1,1,1}; return 1;
}
static int32_t mock_find(void*, const char*) { return 0; }
static int32_t mock_nav(void*, arkheon_vec3, arkheon_vec3 to,
                        arkheon_vec3* path, int32_t /*max*/) {
    path[0] = to; return 1;
}
static void    mock_done(void*, int32_t seq, int32_t res) {
    std::printf("goal %d done, result=%d\n", seq, res);
}
static arkheon_vec3 mock_grav(void*) { return {0, -9.81f, 0}; }

int main() {
    /* Sanity: SDK version handshake */
    std::printf("SDK version = 0x%08x\n", arkheon_character_sdk_version());

    /* Create */
    float segs[10] = {0.0f, 0.0f, 0.296595f, 0.296595f, 0.0f, 0.0f,
                      0.406626f, 0.406626f, 0.433194f, 0.433194f};
    void* h = arkheon_character_create(segs);
    if (!h) { std::printf("FAIL: create\n"); return 1; }

    /* Identity bones, empty input, no goal */
    arkheon_bone_state    bones[66] = {};
    for (auto& b : bones) b.local_rotation = {0,0,0,1};

    arkheon_input_state input = {};
    arkheon_env_api env = { /*ctx*/ nullptr,
        mock_raycast, mock_aabb, mock_find, mock_nav, mock_done, mock_grav };

    /* 1000 ticks at 50 Hz */
    arkheon_frame frame = {};
    frame.delta_time_s = 0.02;
    for (uint64_t i = 0; i < 1000; ++i) {
        frame.simulation_time_s = i * 0.02;
        frame.frame_number = i;
        arkheon_bone_override out[10] = {};
        arkheon_vec3 dt = {0,0,0};
        arkheon_quat dr = {0,0,0,1};
        int rc = arkheon_character_tick(h, &frame, bones, out, &dt, &dr,
                                        &input, /*goal*/ nullptr, &env);
        if (rc != 0) { std::printf("FAIL: tick %llu rc=%d\n", i, rc); return 1; }
        for (auto& o : out) {
            float n2 = o.local_rotation.x*o.local_rotation.x +
                       o.local_rotation.y*o.local_rotation.y +
                       o.local_rotation.z*o.local_rotation.z +
                       o.local_rotation.w*o.local_rotation.w;
            if (!std::isfinite(n2) || n2 < 0.5f || n2 > 1.5f) {
                std::printf("FAIL: bad quaternion at tick %llu\n", i); return 1;
            }
        }
    }
    arkheon_character_destroy(h);
    std::printf("PASS: 1000 ticks, all quaternions finite & unit-ish\n");
    return 0;
}
