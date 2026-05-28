#include "arkheon/character/ICharacterController.h"
#include <cstring>
#include <new>
#include <cmath>

static arkheon_quat quat_identity() {
    return { 0.0f, 0.0f, 0.0f, 1.0f };
}

static arkheon_quat quat_from_axis_angle(float ax, float ay, float az, float angle_rad) {
    float half = angle_rad * 0.5f;
    float s = std::sin(half);
    float c = std::cos(half);
    return { ax * s, ay * s, az * s, c };
}

static arkheon_quat quat_normalize(arkheon_quat q) {
    float n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (n2 <= 0.000001f) {
        return quat_identity();
    }

    float inv = 1.0f / std::sqrt(n2);
    return { q.x * inv, q.y * inv, q.z * inv, q.w * inv };
}

static float clamp_float(float v, float mn, float mx) {
    if (v < mn) return mn;
    if (v > mx) return mx;
    return v;
}

static float distance_xz_to_aabb(arkheon_vec3 p, arkheon_vec3 mn, arkheon_vec3 mx) {
    float cx = clamp_float(p.x, mn.x, mx.x);
    float cz = clamp_float(p.z, mn.z, mx.z);

    float dx = p.x - cx;
    float dz = p.z - cz;

    return std::sqrt(dx * dx + dz * dz);
}

static arkheon_vec3 aabb_center(arkheon_vec3 mn, arkheon_vec3 mx) {
    return {
        (mn.x + mx.x) * 0.5f,
        (mn.y + mx.y) * 0.5f,
        (mn.z + mx.z) * 0.5f
    };
}

namespace {

    constexpr int HID_W = 26;
    constexpr int HID_A = 4;
    constexpr int HID_S = 22;
    constexpr int HID_D = 7;
    constexpr int HID_LSHIFT = 225;

    struct Controller {
        float seg_len[10] = { 0 };
        int32_t last_seq_id = -1;
        int32_t active_motion = 0;  // 0=Q/walk, 1=E/push, 2=R/climb
        arkheon_quat joint_pose[ARK_JOINT_COUNT] = {};

        arkheon_vec3 estimated_root_pos = { 0.0f, 0.0f, 0.0f };
        bool goal_reported = false;
        float action_timer_s = 0.0f;
    };

} // namespace

extern "C" {

    ARKHEON_CHAR_EXPORT uint32_t arkheon_character_sdk_version(void) {
        return ARKHEON_CHARACTER_SDK_VERSION;
    }

    ARKHEON_CHAR_EXPORT const char* arkheon_character_plugin_name(void) {
        return "Hagar Character Plugin v0.1";
    }

    ARKHEON_CHAR_EXPORT void arkheon_character_get_motion_clips(
        void* /*handle*/,
        int32_t out_clip_ids[3])
    {
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

        for (int i = 0; i < ARK_JOINT_COUNT; ++i) {
            c->joint_pose[i] = quat_identity();
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

            float dt = 0.02f;
            if (frame && frame->delta_time_s > 0.0) {
                dt = static_cast<float>(frame->delta_time_s);
            }

            // Detect mission goal changes immediately.
            if (current_goal && current_goal->sequence_id != c->last_seq_id) {
                c->last_seq_id = current_goal->sequence_id;
                c->goal_reported = false;
                c->action_timer_s = 0.0f;
            }

            // Manual hotkey motion selection.
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

            int motion_for_pose = c->active_motion;

            // Default manual WASD movement.
            float move_x = 0.0f;
            float move_z = 0.0f;

            if (input) {
                if (input->keys[HID_W]) move_z += 1.0f;
                if (input->keys[HID_S]) move_z -= 1.0f;
                if (input->keys[HID_D]) move_x += 1.0f;
                if (input->keys[HID_A]) move_x -= 1.0f;
            }

            float len = std::sqrt(move_x * move_x + move_z * move_z);
            if (len > 0.0001f) {
                move_x /= len;
                move_z /= len;
            }

            float yaw = input ? input->look_yaw_rad : 0.0f;
            float sin_yaw = std::sin(yaw);
            float cos_yaw = std::cos(yaw);

            float world_x = move_x * cos_yaw + move_z * sin_yaw;
            float world_z = -move_x * sin_yaw + move_z * cos_yaw;

            float speed = 1.4f;
            if (input && input->keys[HID_LSHIFT]) {
                speed = 3.2f;
            }

            arkheon_vec3 root_delta = {
                world_x * speed * dt,
                0.0f,
                world_z * speed * dt
            };

            // Mission mode: simple GOTO overrides manual WASD movement.
            if (current_goal && current_goal->type == ARK_GOAL_GOTO && !c->goal_reported) {
                float dx = current_goal->target_position.x - c->estimated_root_pos.x;
                float dz = current_goal->target_position.z - c->estimated_root_pos.z;

                float dist = std::sqrt(dx * dx + dz * dz);
                float tolerance = current_goal->tolerance_m > 0.0f ? current_goal->tolerance_m : 0.3f;

                if (dist <= tolerance) {
                    root_delta = { 0.0f, 0.0f, 0.0f };

                    if (env && env->report_goal_complete) {
                        env->report_goal_complete(
                            env->host_ctx,
                            current_goal->sequence_id,
                            ARK_GOAL_RESULT_OK
                        );
                    }

                    c->goal_reported = true;
                }
                else if (dist > 0.0001f) {
                    float mission_speed = 1.2f;
                    float step = mission_speed * dt;

                    if (step > dist) {
                        step = dist;
                    }

                    root_delta = {
                        (dx / dist) * step,
                        0.0f,
                        (dz / dist) * step
                    };

                    c->active_motion = 0;
                    motion_for_pose = 0;
                }
            }

            // Mission mode: simple PUSH.
            // Approach the target object, hold push pose briefly, then report complete.
            if (current_goal && current_goal->type == ARK_GOAL_PUSH && !c->goal_reported) {
                arkheon_vec3 mn = { 0.0f, 0.0f, 0.0f };
                arkheon_vec3 mx = { 0.0f, 0.0f, 0.0f };

                if (env && env->get_object_aabb &&
                    env->get_object_aabb(env->host_ctx, current_goal->target_object_id, &mn, &mx)) {

                    arkheon_vec3 center = aabb_center(mn, mx);

                    float dx = center.x - c->estimated_root_pos.x;
                    float dz = center.z - c->estimated_root_pos.z;
                    float dist_to_box = distance_xz_to_aabb(c->estimated_root_pos, mn, mx);

                    if (dist_to_box > 0.45f) {
                        float dist = std::sqrt(dx * dx + dz * dz);

                        if (dist > 0.0001f) {
                            float approach_speed = 1.1f;
                            float step = approach_speed * dt;

                            if (step > dist) {
                                step = dist;
                            }

                            root_delta = {
                                (dx / dist) * step,
                                0.0f,
                                (dz / dist) * step
                            };

                            c->active_motion = 0;
                            motion_for_pose = 0;
                        }
                    }
                    else {
                        c->active_motion = 1;
                        motion_for_pose = 1;
                        c->action_timer_s += dt;

                        root_delta = {
                            current_goal->push_dir.x * 0.25f * dt,
                            0.0f,
                            current_goal->push_dir.z * 0.25f * dt
                        };

                        if (c->action_timer_s >= 1.0f) {
                            if (env && env->report_goal_complete) {
                                env->report_goal_complete(
                                    env->host_ctx,
                                    current_goal->sequence_id,
                                    ARK_GOAL_RESULT_OK
                                );
                            }

                            c->goal_reported = true;
                            root_delta = { 0.0f, 0.0f, 0.0f };
                        }
                    }
                }
            }

            // Procedural 10-joint target pose.
            float t = 0.0f;
            if (frame) {
                t = static_cast<float>(frame->simulation_time_s);
            }

            arkheon_quat target[ARK_JOINT_COUNT];
            for (int i = 0; i < ARK_JOINT_COUNT; ++i) {
                target[i] = quat_identity();
            }

            if (motion_for_pose == 0) {
                // Walk-like leg swing.
                float swing = std::sin(t * 6.0f) * 0.35f;

                target[ARK_JOINT_THIGH_L] = quat_from_axis_angle(1, 0, 0, swing);
                target[ARK_JOINT_THIGH_R] = quat_from_axis_angle(1, 0, 0, -swing);

                target[ARK_JOINT_CALF_L] = quat_from_axis_angle(1, 0, 0, -std::fabs(swing) * 0.7f);
                target[ARK_JOINT_CALF_R] = quat_from_axis_angle(1, 0, 0, -std::fabs(swing) * 0.7f);

                target[ARK_JOINT_UPPERARM_L] = quat_from_axis_angle(1, 0, 0, -swing * 0.5f);
                target[ARK_JOINT_UPPERARM_R] = quat_from_axis_angle(1, 0, 0, swing * 0.5f);
            }
            else if (motion_for_pose == 1) {
                // Push pose: both arms forward.
                target[ARK_JOINT_UPPERARM_L] = quat_from_axis_angle(1, 0, 0, -0.7f);
                target[ARK_JOINT_UPPERARM_R] = quat_from_axis_angle(1, 0, 0, -0.7f);
                target[ARK_JOINT_LOWERARM_L] = quat_from_axis_angle(1, 0, 0, -0.25f);
                target[ARK_JOINT_LOWERARM_R] = quat_from_axis_angle(1, 0, 0, -0.25f);
            }
            else {
                // Climb-like pose: one leg and both arms raised.
                target[ARK_JOINT_UPPERARM_L] = quat_from_axis_angle(1, 0, 0, -0.9f);
                target[ARK_JOINT_UPPERARM_R] = quat_from_axis_angle(1, 0, 0, -0.9f);
                target[ARK_JOINT_THIGH_L] = quat_from_axis_angle(1, 0, 0, 0.7f);
                target[ARK_JOINT_CALF_L] = quat_from_axis_angle(1, 0, 0, -0.8f);
            }

            // Simplified PD-like smoothing.
            float alpha = 0.18f;
            for (int i = 0; i < ARK_JOINT_COUNT; ++i) {
                c->joint_pose[i].x += (target[i].x - c->joint_pose[i].x) * alpha;
                c->joint_pose[i].y += (target[i].y - c->joint_pose[i].y) * alpha;
                c->joint_pose[i].z += (target[i].z - c->joint_pose[i].z) * alpha;
                c->joint_pose[i].w += (target[i].w - c->joint_pose[i].w) * alpha;

                c->joint_pose[i] = quat_normalize(c->joint_pose[i]);

                out_overrides[i].local_rotation = c->joint_pose[i];
                out_overrides[i].apply = 1;
            }

            *out_root_translation_delta = root_delta;

            // Update simple root estimate.
            c->estimated_root_pos.x += root_delta.x;
            c->estimated_root_pos.y += root_delta.y;
            c->estimated_root_pos.z += root_delta.z;

            // Safe root rotation delta.
            *out_root_rotation_delta = { 0.0f, 0.0f, 0.0f, 1.0f };

            return 0;
        }
        catch (...) {
            return 2;
        }
    }

} // extern "C"