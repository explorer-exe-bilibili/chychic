#include "app_motor.h"
#include "bsp_motor.h"
#include "chassis_config.h"
#include "motor_speed.h"
#include "pid.h"
#include <math.h>
#include <stddef.h>

_Static_assert(MOTOR_LEFT_DEADZONE_PERMILLE >= 0 &&
                   MOTOR_LEFT_DEADZONE_PERMILLE < MOTOR_PWM_MAX_PERMILLE,
               "Left motor deadzone must be in [0, 1000)");
_Static_assert(MOTOR_RIGHT_DEADZONE_PERMILLE >= 0 &&
                   MOTOR_RIGHT_DEADZONE_PERMILLE < MOTOR_PWM_MAX_PERMILLE,
               "Right motor deadzone must be in [0, 1000)");

/* motor_deadzone_permille：每个轮子的启动补偿，单位为千分比。 */
static const int32_t motor_deadzone_permille[MOTOR_COUNT] = {MOTOR_LEFT_DEADZONE_PERMILLE,
                                                             MOTOR_RIGHT_DEADZONE_PERMILLE};

/** wheel_speed_controller_t：单轮 PID 历史和上次控制的目标方向。 */
typedef struct {
    pid_state_t pid;
    int32_t target_direction; /* 1 为正转，-1 为反转，0 为已复位。 */
} wheel_speed_controller_t;

/* wheel_speed_controllers：按电机编号保存互相独立的控制状态。 */
static wheel_speed_controller_t wheel_speed_controllers[MOTOR_COUNT];

/* 可在线调整，初值由底盘配置提供；全零时只输出前馈。 */
static pid_gains_t wheel_speed_pid_gains[MOTOR_COUNT] = {
    [MOTOR_LEFT] = MOTOR_LEFT_SPEED_PID_GAINS,
    [MOTOR_RIGHT] = MOTOR_RIGHT_SPEED_PID_GAINS,
};

/** wheel_speed_reset_all：清除双轮 PID 历史和方向。参数：无。返回：无。 */
static void wheel_speed_reset_all(void) {
    for (motor_id_t motor_id = MOTOR_LEFT; motor_id < MOTOR_COUNT; ++motor_id) {
        wheel_speed_controllers[motor_id] = (wheel_speed_controller_t){0};
    }
}

bool app_motor_init(void) {
    wheel_speed_reset_all();
    return bsp_motor_init();
}

bool app_motor_get_speed_pid(motor_id_t motor_id, pid_gains_t *gains) {
    if ((unsigned int)motor_id >= MOTOR_COUNT || gains == NULL) {
        return false;
    }
    *gains = wheel_speed_pid_gains[motor_id];
    return true;
}

bool app_motor_set_speed_pid(motor_id_t motor_id, const pid_gains_t *gains) {
    if ((unsigned int)motor_id >= MOTOR_COUNT || gains == NULL || !isfinite(gains->kp) ||
        !isfinite(gains->ki) || !isfinite(gains->kd) || gains->kp < 0.0f || gains->ki < 0.0f ||
        gains->kd < 0.0f) {
        return false;
    }

    pid_gains_t *current_gains = &wheel_speed_pid_gains[motor_id];
    if (gains->kp != current_gains->kp || gains->ki != current_gains->ki ||
        gains->kd != current_gains->kd) {
        *current_gains = *gains;
        wheel_speed_controllers[motor_id] = (wheel_speed_controller_t){0};
    }
    return true;
}

int32_t app_motor_control_step(motor_id_t motor_id, float target_speed_mm_s,
                               float measured_speed_mm_s, float dt_s) {
    if ((unsigned int)motor_id >= MOTOR_COUNT) {
        return 0;
    }

    wheel_speed_controller_t *controller = &wheel_speed_controllers[motor_id];
    float speed_magnitude_mm_s = fabsf(target_speed_mm_s);
    if (!isfinite(target_speed_mm_s) || !isfinite(measured_speed_mm_s) || !isfinite(dt_s) ||
        dt_s <= 0.0f || speed_magnitude_mm_s < MOTOR_ZERO_SPEED_THRESHOLD_MM_S) {
        *controller = (wheel_speed_controller_t){0};
        return 0;
    }
    if (speed_magnitude_mm_s > CHASSIS_MAX_WHEEL_SPEED_MM_S) {
        speed_magnitude_mm_s = CHASSIS_MAX_WHEEL_SPEED_MM_S;
    }

    /* 保留原开环映射的四舍五入，以带符号 PWM 千分比作为前馈。 */
    int32_t deadzone_permille = motor_deadzone_permille[motor_id];
    int32_t target_direction = target_speed_mm_s >= 0.0f ? 1 : -1;
    int32_t feedforward_permille =
        deadzone_permille + (int32_t)((MOTOR_PWM_MAX_PERMILLE - deadzone_permille) *
                                          speed_magnitude_mm_s / CHASSIS_MAX_WHEEL_SPEED_MM_S +
                                      0.5f);
    feedforward_permille *= target_direction;
    const pid_gains_t *gains = &wheel_speed_pid_gains[motor_id];

    if (controller->target_direction != target_direction) {
        pid_reset(&controller->pid);
        controller->target_direction = target_direction;
    }
    float error_mm_s = (float)target_direction * speed_magnitude_mm_s - measured_speed_mm_s;
    float output_min = target_direction > 0 ? 0.0f : -(float)MOTOR_PWM_MAX_PERMILLE;
    float output_max = target_direction > 0 ? (float)MOTOR_PWM_MAX_PERMILLE : 0.0f;
    float output_permille;
    if (!pid_step(&controller->pid, gains, error_mm_s, dt_s, (float)feedforward_permille,
                  output_min, output_max, &output_permille)) {
        return 0;
    }
    return (int32_t)lroundf(output_permille);
}

void app_motor_update(float left_target_mm_s, float right_target_mm_s, uint32_t dt_ms) {
    if (dt_ms == 0U) {
        return;
    }
    const float target_speed_mm_s[MOTOR_COUNT] = {left_target_mm_s, right_target_mm_s};
    float dt_s = (float)dt_ms / 1000.0f;
    for (motor_id_t motor_id = MOTOR_LEFT; motor_id < MOTOR_COUNT; ++motor_id) {
        motor_feedback_t feedback;
        if (!motor_speed_get_feedback(motor_id, &feedback)) {
            app_motor_stop();
            return;
        }
        int32_t pwm_permille = app_motor_control_step(motor_id, target_speed_mm_s[motor_id],
                                                      feedback.filtered_speed_mm_s, dt_s);
        bsp_motor_set_pwm(motor_id, pwm_permille);
    }
}

void app_motor_stop(void) {
    wheel_speed_reset_all();
    bsp_motor_stop_all();
}
