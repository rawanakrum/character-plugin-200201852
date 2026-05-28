#include "arkheon/character/ICharacterController.h"
#include <cstring>
#include <new>
#include <cmath>

namespace {
    constexpr int HID_W = 26;
    constexpr int HID_A = 4;
    constexpr int HID_S = 22;
    constexpr int HID_D = 7;
    constexpr int HID_LSHIFT = 225;

    constexpr float WALK_SPEED_MPS = 1.4f;
    constexpr float SPRINT_SPEED_MPS = 4.0f;

static arkheon_quat yaw_to_quat(float yaw_rad) {
        const float half = yaw_rad * 0.5f;
        return { 0.0f, std::sin(half), 0.0f, std::cos(half) };
    }
    
static float distance_xz(arkheon_vec3 a, arkheon_vec3 b) {
    const float dx = b.x - a.x;
    const float dz = b.z - a.z;
    return std::sqrt(dx * dx + dz * dz);
}

static arkheon_vec3 normalize_xz_direction(arkheon_vec3 from, arkheon_vec3 to) {
    arkheon_vec3 dir = {
        to.x - from.x,
        0.0f,
        to.z - from.z
    };

    const float len_sq = dir.x * dir.x + dir.z * dir.z;
    if (len_sq < 0.0001f) {
        return { 0.0f, 0.0f, 0.0f };
    }

    const float inv_len = 1.0f / std::sqrt(len_sq);
    dir.x *= inv_len;
    dir.z *= inv_len;
    return dir;
}

struct Controller {
    float seg_len[10] = { 0 };
    int32_t last_seq_id = -1;
    int32_t active_motion = 0;  // 0=Q, 1=E, 2=R

    arkheon_vec3 estimated_root_pos = { 0.0f, 0.0f, 0.0f };
    uint8_t reported_current_goal = 0;
};

} // namespace

extern "C" {

ARKHEON_CHAR_EXPORT uint32_t arkheon_character_sdk_version(void) {
    return ARKHEON_CHARACTER_SDK_VERSION;
}

ARKHEON_CHAR_EXPORT const char* arkheon_character_plugin_name(void) {
    return "Rawan Character Plugin v0.1";
}

ARKHEON_CHAR_EXPORT void arkheon_character_get_motion_clips(
    void* /*handle*/,
    int32_t out_clip_ids[3])
{
    // Motion A, B, C
    out_clip_ids[0] = 12;  // walk_forward
    out_clip_ids[1] = 47;  // push_two_handed
    out_clip_ids[2] = 83;  // climb_low_step
}

ARKHEON_CHAR_EXPORT void* arkheon_character_create(
    const float segment_lengths_m[10])
{
    Controller* c = new (std::nothrow) Controller();
    if (!c) {
        return nullptr;
    }

    if (segment_lengths_m) {
        std::memcpy(c->seg_len, segment_lengths_m, sizeof(c->seg_len));
    }

    return c;
}

ARKHEON_CHAR_EXPORT void arkheon_character_destroy(void* handle) {
    Controller* c = static_cast<Controller*>(handle);
    delete c;
}

ARKHEON_CHAR_EXPORT int32_t arkheon_character_tick(
    void* handle,
    const arkheon_frame* frame, 
    const arkheon_bone_state /*in_bones*/[66],
    arkheon_bone_override out_overrides[10],
    arkheon_vec3* out_root_translation_delta,
    arkheon_quat* out_root_rotation_delta,
    const arkheon_input_state* input,
    const arkheon_mission_goal* current_goal,
    const arkheon_env_api* env)
{
    try {
        if (!handle || !out_overrides || !out_root_translation_delta || !out_root_rotation_delta) {
            return 1;
        }

        Controller* c = static_cast<Controller*>(handle);

        // Safe default: do not override joints yet.
        for (int i = 0; i < ARK_JOINT_COUNT; ++i) {
            out_overrides[i].local_rotation = {0.0f, 0.0f, 0.0f, 1.0f};
            out_overrides[i].apply = 0;
        }

       
       // Safe default: no movement.
        *out_root_translation_delta = { 0.0f, 0.0f, 0.0f };
        *out_root_rotation_delta = { 0.0f, 0.0f, 0.0f, 1.0f };

        // Basic WASD locomotion.
        // The host integrates this root delta and handles world collision.
        if (input && frame && !frame->is_paused) {
            float move_x = 0.0f;
            float move_z = 0.0f;

            if (input->keys[HID_W]) move_z += 1.0f;
            if (input->keys[HID_S]) move_z -= 1.0f;
            if (input->keys[HID_D]) move_x += 1.0f;
            if (input->keys[HID_A]) move_x -= 1.0f;

            const float len_sq = move_x * move_x + move_z * move_z;

            if (len_sq > 0.0001f) {
                const float inv_len = 1.0f / std::sqrt(len_sq);
                move_x *= inv_len;
                move_z *= inv_len;

                const float speed = input->keys[HID_LSHIFT]
                    ? SPRINT_SPEED_MPS
                    : WALK_SPEED_MPS;

                const float dt = static_cast<float>(frame->delta_time_s);
                const float yaw = input->look_yaw_rad;

                const float sin_yaw = std::sin(yaw);
                const float cos_yaw = std::cos(yaw);

                // Local movement rotated into world direction.
                const float world_x = move_x * cos_yaw + move_z * sin_yaw;
                const float world_z = -move_x * sin_yaw + move_z * cos_yaw;

                *out_root_translation_delta = {
                    world_x * speed * dt,
                    0.0f,
                    world_z * speed * dt
                };

                *out_root_rotation_delta = yaw_to_quat(yaw);
            }
        }

        // Hotkeys only change internal active motion for now.
        if (input) {
            if (input->hotkey_motion_a) {
                c->active_motion = 0;
            }
            if (input->hotkey_motion_b) {
                c->active_motion = 1;
            }
            if (input->hotkey_motion_c) {
                c->active_motion = 2;
            }
        }

        // Detect mission goal changes.
        if (current_goal && current_goal->sequence_id != c->last_seq_id) {
            c->last_seq_id = current_goal->sequence_id;
            c->reported_current_goal = 0;

            // First simple version: assume spawn/root starts near origin.
            // Later we can improve this if APP gives us root state.
            c->estimated_root_pos = { 0.0f, 0.0f, 0.0f };
        }

        // Basic mission GOTO handling.
        // If a mission is active, it controls root movement instead of manual WASD.
        if (current_goal &&
            current_goal->type == ARK_GOAL_GOTO &&
            env &&
            env->report_goal_complete &&
            frame &&
            !frame->is_paused &&
            !c->reported_current_goal) {

            const float tolerance =
                current_goal->tolerance_m > 0.0f ? current_goal->tolerance_m : 0.3f;

            const float dist = distance_xz(
                c->estimated_root_pos,
                current_goal->target_position
            );

            if (dist <= tolerance) {
                env->report_goal_complete(
                    env->host_ctx,
                    current_goal->sequence_id,
                    ARK_GOAL_RESULT_OK
                );
                c->reported_current_goal = 1;
            }
            else {
                const float dt = static_cast<float>(frame->delta_time_s);
                const float step = WALK_SPEED_MPS * dt;

                arkheon_vec3 dir = normalize_xz_direction(
                    c->estimated_root_pos,
                    current_goal->target_position
                );

                arkheon_vec3 delta = {
                    dir.x * step,
                    0.0f,
                    dir.z * step
                };

                // Do not overshoot the target.
                if (step > dist) {
                    delta = {
                        current_goal->target_position.x - c->estimated_root_pos.x,
                        0.0f,
                        current_goal->target_position.z - c->estimated_root_pos.z
                    };
                }

                *out_root_translation_delta = delta;

                c->estimated_root_pos.x += delta.x;
                c->estimated_root_pos.y += delta.y;
                c->estimated_root_pos.z += delta.z;

                const float yaw = std::atan2(dir.x, dir.z);
                *out_root_rotation_delta = yaw_to_quat(yaw);
            }
        }

        return 0;
    }
    catch (...) {
        // Never allow exceptions to cross the ABI boundary.
        return 2;
    }
}

} // extern "C"
