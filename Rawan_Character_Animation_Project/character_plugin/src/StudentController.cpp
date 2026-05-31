#include "arkheon/character/ICharacterController.h"

// No STL/CRT dependency version: keeps the DLL easy for N8RO to load.
// Plugin identity is intentionally consistent with the exported DLL.

#if defined(_WIN32) && defined(ARKHEON_NO_CRT)
extern "C" int _fltused = 0;
#endif

static arkheon_quat quat_identity() {
    arkheon_quat q = { 0.0f, 0.0f, 0.0f, 1.0f };
    return q;
}

static float ark_fabs(float v) {
    return v < 0.0f ? -v : v;
}

static float ark_sqrt(float v) {
    if (v <= 0.0f) {
        return 0.0f;
    }

    float x = v > 1.0f ? v : 1.0f;
    for (int i = 0; i < 8; ++i) {
        x = 0.5f * (x + v / x);
    }
    return x;
}

static float ark_wrap_pi(float x) {
    const float pi = 3.14159265358979323846f;
    const float two_pi = 6.28318530717958647692f;

    int k = (int)(x / two_pi);
    x -= (float)k * two_pi;

    while (x > pi) {
        x -= two_pi;
    }
    while (x < -pi) {
        x += two_pi;
    }
    return x;
}

static float ark_sin(float x) {
    x = ark_wrap_pi(x);
    float x2 = x * x;

    // 7th-order Taylor approximation. Good enough for visible motion blending.
    return x * (1.0f
        - x2 * (1.0f / 6.0f)
        + x2 * x2 * (1.0f / 120.0f)
        - x2 * x2 * x2 * (1.0f / 5040.0f));
}

static float ark_cos(float x) {
    return ark_sin(x + 1.57079632679489661923f);
}

static arkheon_quat quat_from_axis_angle(float ax, float ay, float az, float angle_rad) {
    float half = angle_rad * 0.5f;
    float s = ark_sin(half);
    float c = ark_cos(half);

    arkheon_quat q = { ax * s, ay * s, az * s, c };
    return q;
}

static arkheon_quat quat_normalize(arkheon_quat q) {
    float n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (n2 <= 0.000001f) {
        return quat_identity();
    }

    float inv = 1.0f / ark_sqrt(n2);
    arkheon_quat out = { q.x * inv, q.y * inv, q.z * inv, q.w * inv };
    return out;
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

    return ark_sqrt(dx * dx + dz * dz);
}

static arkheon_vec3 aabb_center(arkheon_vec3 mn, arkheon_vec3 mx) {
    arkheon_vec3 out = {
        (mn.x + mx.x) * 0.5f,
        (mn.y + mx.y) * 0.5f,
        (mn.z + mx.z) * 0.5f
    };
    return out;
}

static void copy_segment_lengths(float dst[10], const float* src) {
    for (int i = 0; i < 10; ++i) {
        dst[i] = src ? src[i] : 0.0f;
    }
}

namespace {

    constexpr int HID_W = 26;
    constexpr int HID_A = 4;
    constexpr int HID_S = 22;
    constexpr int HID_D = 7;
    constexpr int HID_LSHIFT = 225;

    struct Controller {
        uint8_t in_use;
        float seg_len[10];
        int32_t last_seq_id;
        int32_t active_motion;  // 0=walk-like, 1=push-like, 2=climb-like
        arkheon_quat joint_pose[ARK_JOINT_COUNT];

        arkheon_vec3 estimated_root_pos;
        uint8_t goal_reported;
        float action_timer_s;
    };

    constexpr int MAX_CONTROLLERS = 16;
    static Controller g_controllers[MAX_CONTROLLERS];

    static void reset_controller(Controller* c, const float segment_lengths_m[10]) {
        c->in_use = 1;
        copy_segment_lengths(c->seg_len, segment_lengths_m);
        c->last_seq_id = -1;
        c->active_motion = 0;
        c->estimated_root_pos = { 0.0f, 0.0f, 0.0f };
        c->goal_reported = 0;
        c->action_timer_s = 0.0f;

        for (int i = 0; i < ARK_JOINT_COUNT; ++i) {
            c->joint_pose[i] = quat_identity();
        }
    }

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
        if (!out_clip_ids) {
            return;
        }

        out_clip_ids[0] = 12;  // walk_forward
        out_clip_ids[1] = 47;  // push_two_handed
        out_clip_ids[2] = 83;  // climb_low_step
    }

    ARKHEON_CHAR_EXPORT void* arkheon_character_create(
        const float segment_lengths_m[10])
    {
        for (int i = 0; i < MAX_CONTROLLERS; ++i) {
            if (!g_controllers[i].in_use) {
                reset_controller(&g_controllers[i], segment_lengths_m);
                return &g_controllers[i];
            }
        }

        return 0;
    }

    ARKHEON_CHAR_EXPORT void arkheon_character_destroy(void* handle) {
        Controller* c = (Controller*)handle;
        if (!c) {
            return;
        }

        for (int i = 0; i < MAX_CONTROLLERS; ++i) {
            if (c == &g_controllers[i]) {
                c->in_use = 0;
                return;
            }
        }
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
        if (!handle || !out_overrides || !out_root_translation_delta || !out_root_rotation_delta) {
            return 1;
        }

        Controller* c = (Controller*)handle;
        if (!c->in_use) {
            return 1;
        }

        float dt = 0.02f;
        if (frame && frame->delta_time_s > 0.0) {
            dt = (float)frame->delta_time_s;
        }

        // Detect mission goal changes immediately.
        if (current_goal && current_goal->sequence_id != c->last_seq_id) {
            c->last_seq_id = current_goal->sequence_id;
            c->goal_reported = 0;
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

        float len = ark_sqrt(move_x * move_x + move_z * move_z);
        if (len > 0.0001f) {
            move_x /= len;
            move_z /= len;
        }

        float yaw = input ? input->look_yaw_rad : 0.0f;
        float sin_yaw = ark_sin(yaw);
        float cos_yaw = ark_cos(yaw);

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

            float dist = ark_sqrt(dx * dx + dz * dz);
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

                c->goal_reported = 1;
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
                    float dist = ark_sqrt(dx * dx + dz * dz);

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

                        c->goal_reported = 1;
                        root_delta = { 0.0f, 0.0f, 0.0f };
                    }
                }
            }
        }

        // Mission mode: simple CLIMB.
        // Approach the object, hold climb pose, move slightly upward/forward, then report complete.
        if (current_goal && current_goal->type == ARK_GOAL_CLIMB && !c->goal_reported) {
            arkheon_vec3 mn = { 0.0f, 0.0f, 0.0f };
            arkheon_vec3 mx = { 0.0f, 0.0f, 0.0f };

            if (env && env->get_object_aabb &&
                env->get_object_aabb(env->host_ctx, current_goal->target_object_id, &mn, &mx)) {

                arkheon_vec3 center = aabb_center(mn, mx);

                float dx = center.x - c->estimated_root_pos.x;
                float dz = center.z - c->estimated_root_pos.z;
                float dist_to_box = distance_xz_to_aabb(c->estimated_root_pos, mn, mx);

                if (dist_to_box > 0.35f) {
                    float dist = ark_sqrt(dx * dx + dz * dz);
                    if (dist > 0.0001f) {
                        float approach_speed = 1.0f;
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
                    c->active_motion = 2;
                    motion_for_pose = 2;
                    c->action_timer_s += dt;

                    float top_y = mx.y;
                    float climb_y = top_y - c->estimated_root_pos.y;
                    climb_y = clamp_float(climb_y, 0.0f, 0.9f);

                    root_delta = {
                        dx * 0.25f * dt,
                        climb_y * 1.2f * dt,
                        dz * 0.25f * dt
                    };

                    if (c->action_timer_s >= 1.2f) {
                        if (env && env->report_goal_complete) {
                            env->report_goal_complete(
                                env->host_ctx,
                                current_goal->sequence_id,
                                ARK_GOAL_RESULT_OK
                            );
                        }

                        c->goal_reported = 1;
                        root_delta = { 0.0f, 0.0f, 0.0f };
                    }
                }
            }
        }

        // Procedural 10-joint target pose.
        float t = 0.0f;
        if (frame) {
            t = (float)frame->simulation_time_s;
        }

        arkheon_quat target[ARK_JOINT_COUNT];
        for (int i = 0; i < ARK_JOINT_COUNT; ++i) {
            target[i] = quat_identity();
        }

        if (motion_for_pose == 0) {
            // Walk-like leg swing.
            float swing = ark_sin(t * 6.0f) * 0.35f;

            target[ARK_JOINT_THIGH_L] = quat_from_axis_angle(1, 0, 0, swing);
            target[ARK_JOINT_THIGH_R] = quat_from_axis_angle(1, 0, 0, -swing);

            target[ARK_JOINT_CALF_L] = quat_from_axis_angle(1, 0, 0, -ark_fabs(swing) * 0.7f);
            target[ARK_JOINT_CALF_R] = quat_from_axis_angle(1, 0, 0, -ark_fabs(swing) * 0.7f);

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

} // extern "C"
