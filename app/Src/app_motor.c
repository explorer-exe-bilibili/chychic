#include "app_motor.h"
#include "bsp_motor.h"
#include "chassis_config.h"
#include "motor_speed.h"
#include <math.h>

/* motor_deadzone_permille：每个轮子的开环启动补偿，单位为千分比。 */
static const int32_t motor_deadzone_permille[MOTOR_COUNT] =
              {MOTOR_LEFT_DEADZONE_PERMILLE,MOTOR_RIGHT_DEADZONE_PERMILLE};
/**编译时的检查*/
_Static_assert(MOTOR_LEFT_DEADZONE_PERMILLE >= 0 &&MOTOR_LEFT_DEADZONE_PERMILLE < MOTOR_PWM_MAX_PERMILLE,
               "Left motor deadzone must be in [0, 1000)");
_Static_assert(MOTOR_RIGHT_DEADZONE_PERMILLE >= 0 &&
                   MOTOR_RIGHT_DEADZONE_PERMILLE < MOTOR_PWM_MAX_PERMILLE,
               "Right motor deadzone must be in [0, 1000)");

bool app_motor_init(void) {
    return bsp_motor_init();
}

int32_t app_motor_control_step(motor_id_t motor_id, float target_speed_mm_s/*本周期目标速度，单位mm/s。*/,
                               float measured_speed_mm_s/*实测轮速，单位 mm/s*/, float dt_s/*周期时间，单位秒。*/) {
    if ((unsigned int)motor_id >= MOTOR_COUNT || !isfinite(target_speed_mm_s) ||
        !isfinite(measured_speed_mm_s) || !isfinite(dt_s) || dt_s <= 0.0f) {
        return 0;
    }//检查输入

    /*
     * PID 接入口：左右轮状态按 motor_id 分开，目标、反馈和秒单位周期已传入。
     * 当前开环仅使用目标速度，不计算速度误差；添加 PID 后输出仍需限幅。
     */
    float speed_magnitude_mm_s = fabsf(target_speed_mm_s);//取速度绝对值
    if (speed_magnitude_mm_s < MOTOR_ZERO_SPEED_THRESHOLD_MM_S) {
        return 0;
    }
    if (speed_magnitude_mm_s > CHASSIS_MAX_WHEEL_SPEED_MM_S) {
        speed_magnitude_mm_s = CHASSIS_MAX_WHEEL_SPEED_MM_S;
    }
    int32_t deadzone_permille = motor_deadzone_permille[motor_id];
    int32_t pwm_permille =
        deadzone_permille + (int32_t)((MOTOR_PWM_MAX_PERMILLE - deadzone_permille) *
                                          speed_magnitude_mm_s / CHASSIS_MAX_WHEEL_SPEED_MM_S +
                                      0.5f);//四舍五入
    return target_speed_mm_s >= 0.0f ? pwm_permille : -pwm_permille;//恢复方向
}

void app_motor_update(float left_target_mm_s, float right_target_mm_s, uint32_t dt_ms) {
    if (dt_ms == 0U) {
        return;
    }
    const float target_speed_mm_s[MOTOR_COUNT] = {left_target_mm_s, right_target_mm_s};
    float dt_s = (float)dt_ms / 1000.0f;
    for (motor_id_t motor_id = MOTOR_LEFT; motor_id < MOTOR_COUNT; ++motor_id)
    {
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
    /* PID 接入后在此清除积分、历史误差等状态，防止再次启动时残留控制量。 */
    bsp_motor_stop_all();
}
