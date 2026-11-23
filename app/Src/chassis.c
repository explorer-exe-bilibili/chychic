#include "chassis.h"
#include "app_motor.h"
#include "chassis_config.h"
#include "motor_speed.h"
#include <math.h>

/* target_wheel_speed_mm_s：解算和限幅后的目标轮速，单位毫米/秒。 */
static float target_wheel_speed_mm_s[MOTOR_COUNT];
/* ramped_wheel_speed_mm_s：经过加速度限制的轮速指令，单位毫米/秒。 */
static float ramped_wheel_speed_mm_s[MOTOR_COUNT];
static bool chassis_initialized; /* 电机和编码器是否均已成功启动。 */

/**
 * chassis_ramp_speed：将一个轮子的指令向目标推进，单次变化不超过 max_step_mm_s。
 * current_speed_mm_s：当前指令；target_speed_mm_s：目标；max_step_mm_s：本周期上限。
 * 返回：新的轮速指令，单位毫米/秒。
 */
static float chassis_ramp_speed(float current_speed_mm_s, float target_speed_mm_s,
                                float max_step_mm_s) {
    float speed_difference_mm_s = target_speed_mm_s - current_speed_mm_s;
    if (speed_difference_mm_s > max_step_mm_s) {
        speed_difference_mm_s = max_step_mm_s;
    } else if (speed_difference_mm_s < -max_step_mm_s) {
        speed_difference_mm_s = -max_step_mm_s;
    }
    return current_speed_mm_s + speed_difference_mm_s;
}

bool chassis_init(void) {
    chassis_initialized = false;
    for (motor_id_t motor_id = MOTOR_LEFT; motor_id < MOTOR_COUNT; ++motor_id) {
        target_wheel_speed_mm_s[motor_id] = 0.0f;
        ramped_wheel_speed_mm_s[motor_id] = 0.0f;
    }
    if (!app_motor_init() || !motor_speed_init()) {
        app_motor_stop();
        return false;
    }
    chassis_initialized = true;
    return true;
}

void chassis_set_wheel_speeds(float left_speed_mm_s, float right_speed_mm_s) {
    if (!isfinite(left_speed_mm_s) || !isfinite(right_speed_mm_s)) {
        chassis_stop();
        return;
    }//输入限制
    float largest_speed_mm_s = fmaxf(fabsf(left_speed_mm_s), fabsf(right_speed_mm_s));
    if (largest_speed_mm_s > CHASSIS_MAX_WHEEL_SPEED_MM_S) {
        float scale_factor = CHASSIS_MAX_WHEEL_SPEED_MM_S / largest_speed_mm_s;
        left_speed_mm_s *= scale_factor;
        right_speed_mm_s *= scale_factor;
    }
    target_wheel_speed_mm_s[MOTOR_LEFT] = left_speed_mm_s;
    target_wheel_speed_mm_s[MOTOR_RIGHT] = right_speed_mm_s;
}
/* chassis_set_velocity：将线速度和角速度解算为左右轮目标速度。 */
void chassis_set_velocity(float linear_speed_mm_s, float angular_speed_rad_s) {
    float turn_speed_offset_mm_s = angular_speed_rad_s * CHASSIS_TRACK_WIDTH_MM * 0.5f;
    chassis_set_wheel_speeds(linear_speed_mm_s - turn_speed_offset_mm_s,
                             linear_speed_mm_s + turn_speed_offset_mm_s);
}
/** chassis_update：主循环调用，更新轮速采样、斜坡处理和电机输出。
 * dt_ms：距上次采样的实际毫秒数，0 时不更新。返回：无。
 */
void chassis_update(uint32_t dt_ms) {
    if (!chassis_initialized || dt_ms == 0U) {
        return;
    }
    motor_speed_update(dt_ms);
    float max_step_mm_s = CHASSIS_MAX_ACCELERATION_MM_S2 * (float)dt_ms / 1000.0f;
    for (motor_id_t motor_id = MOTOR_LEFT; motor_id < MOTOR_COUNT; ++motor_id)
    {
        ramped_wheel_speed_mm_s[motor_id] = chassis_ramp_speed(
            ramped_wheel_speed_mm_s[motor_id], target_wheel_speed_mm_s[motor_id], max_step_mm_s);
    }
    app_motor_update(ramped_wheel_speed_mm_s[MOTOR_LEFT], ramped_wheel_speed_mm_s[MOTOR_RIGHT],dt_ms);
}

void chassis_request_stop(void) {
    chassis_set_wheel_speeds(0.0f, 0.0f);
}

void chassis_stop(void) {
    for (motor_id_t motor_id = MOTOR_LEFT; motor_id < MOTOR_COUNT; ++motor_id)
    {
        target_wheel_speed_mm_s[motor_id] = 0.0f;
        ramped_wheel_speed_mm_s[motor_id] = 0.0f;
    }
    if (chassis_initialized)
    {
        app_motor_stop();
    }
}
