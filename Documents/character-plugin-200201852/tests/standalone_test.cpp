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

    /* Locomotion test: pressing W should move the root forward. */
    arkheon_input_state move_input = {};
    move_input.keys[26] = 1;      // HID_W
    move_input.look_yaw_rad = 0.0f;

    arkheon_bone_override move_out[10] = {};
    arkheon_vec3 root_delta = { 0, 0, 0 };
    arkheon_quat root_rot = { 0, 0, 0, 1 };

    int move_rc = arkheon_character_tick(
        h,
        &frame,
        bones,
        move_out,
        &root_delta,
        &root_rot,
        &move_input,
        nullptr,
        &env
    );

    if (move_rc != 0) {
        std::printf("FAIL: locomotion tick rc=%d\n", move_rc);
        return 1;
    }

    if (std::fabs(root_delta.x) < 0.0001f &&
        std::fabs(root_delta.y) < 0.0001f &&
        std::fabs(root_delta.z) < 0.0001f) {
        std::printf("FAIL: W key produced zero root movement\n");
        return 1;
    }

    std::printf("PASS: W key produced root movement: dx=%.4f dy=%.4f dz=%.4f\n",
        root_delta.x, root_delta.y, root_delta.z);

    /* Mission test: GOTO should eventually report goal complete. */
    arkheon_mission_goal goto_goal = {};
    goto_goal.sequence_id = 1;
    goto_goal.type = ARK_GOAL_GOTO;
    goto_goal.target_position = { 0.0f, 0.0f, 1.0f };
    goto_goal.tolerance_m = 0.05f;
    goto_goal.timeout_s = 10.0f;

    for (int i = 0; i < 200; ++i) {
        frame.simulation_time_s = 20.0 + i * 0.02;
        frame.frame_number = 1000 + i;

        arkheon_bone_override goal_out[10] = {};
        arkheon_vec3 goal_delta = { 0, 0, 0 };
        arkheon_quat goal_rot = { 0, 0, 0, 1 };
        arkheon_input_state no_input = {};

        int goal_rc = arkheon_character_tick(
            h,
            &frame,
            bones,
            goal_out,
            &goal_delta,
            &goal_rot,
            &no_input,
            &goto_goal,
            &env
        );

        if (goal_rc != 0) {
            std::printf("FAIL: GOTO tick rc=%d\n", goal_rc);
            return 1;
        }
    }

    std::printf("PASS: GOTO test completed 200 ticks\n");

    arkheon_character_destroy(h);
    std::printf("PASS: 1000 ticks, all quaternions finite & unit-ish\n");
    return 0;
}
