#include "app_motor.h"
#include "bsp_motor.h"
#include "bsp_motor_encoder.h"
#include "chassis.h"
#include "chassis_config.h"
#include "motor_speed.h"
#include "tim.h"
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

/* timer_registers：TIM2、TIM3、TIM4 的内存寄存器替身。 */
static TIM_TypeDef timer_registers[3];
TIM_HandleTypeDef htim2 = {&timer_registers[0]};
TIM_HandleTypeDef htim3 = {&timer_registers[1]};
TIM_HandleTypeDef htim4 = {&timer_registers[2]};
static bool fail_pwm_start;     /* 是否模拟 PWM 启动失败。 */
static bool fail_encoder_start; /* 是否模拟编码器启动失败。 */

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel) {
    (void)timer;
    (void)channel;
    return fail_pwm_start ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channel) {
    (void)timer;
    (void)channel;
    return fail_encoder_start ? HAL_ERROR : HAL_OK;
}

/** assert_close：断言浮点值接近期望值。actual/expected 为待比较值；返回：无。 */
static void assert_close(float actual, float expected) {
    assert(fabsf(actual - expected) < 0.001f);
}

/** reset_fixture：恢复定时器配置并初始化整套底盘。参数：无。返回：无。 */
static void reset_fixture(void) {
    for (unsigned int index = 0; index < 3; ++index) {
        timer_registers[index] = (TIM_TypeDef){0};
    }
    htim2.Instance->ARR = 65535U;
    htim3.Instance->ARR = 65535U;
    htim4.Instance->ARR = 99U;
    fail_pwm_start = false;
    fail_encoder_start = false;
    assert(chassis_init());
}

/** assert_stopped：确认四个电机输出均为零。参数：无。返回：无。 */
static void assert_stopped(void) {
    for (unsigned int channel = 0; channel < 4; ++channel) {
        assert(htim4.Instance->compare[channel] == 0U);
    }
}

/** test_pwm_scaling：回归千分比/ARR 错配、方向和极限输入。参数：无。返回：无。 */
static void test_pwm_scaling(void) {
    reset_fixture();
    bsp_motor_set_pwm(MOTOR_LEFT, 180);
    assert(htim4.Instance->compare[0] == 18U);
    assert(htim4.Instance->compare[1] == 0U);
    bsp_motor_set_pwm(MOTOR_LEFT, -590);
    assert(htim4.Instance->compare[0] == 0U);
    assert(htim4.Instance->compare[1] == 59U);
    bsp_motor_set_pwm(MOTOR_RIGHT, 1000);
    assert(htim4.Instance->compare[2] == 100U);
    assert(htim4.Instance->compare[3] == 0U);
    bsp_motor_set_pwm(MOTOR_RIGHT, INT_MIN);
    assert(htim4.Instance->compare[2] == 0U);
    assert(htim4.Instance->compare[3] == 100U);
    bsp_motor_set_pwm(MOTOR_LEFT, INT_MAX);
    assert(htim4.Instance->compare[0] == 100U);
    bsp_motor_set_pwm((motor_id_t)-1, 500);
    bsp_motor_set_pwm(MOTOR_COUNT, 500);
    assert(htim4.Instance->compare[0] == 100U);

    htim4.Instance->ARR = 999U;
    bsp_motor_set_pwm(MOTOR_LEFT, 180);
    assert(htim4.Instance->compare[0] == 180U);
    htim4.Instance->CR1 = TIM_CR1_UDIS | 1U;
    bsp_motor_set_pwm(MOTOR_LEFT, 500);
    assert(htim4.Instance->CR1 == (TIM_CR1_UDIS | 1U));
    htim4.Instance->CR1 = 1U;
    bsp_motor_set_pwm(MOTOR_LEFT, 500);
    assert(htim4.Instance->CR1 == 1U);
    htim4.Instance->ARR = 65535U;
    bsp_motor_set_pwm(MOTOR_LEFT, 1000);
    assert(htim4.Instance->compare[0] == 65535U);
    bsp_motor_stop_all();
    assert_stopped();
}

/** test_encoder_feedback：验证正反回绕、缓存读取和统一复位。参数：无。返回：无。 */
static void test_encoder_feedback(void) {
    reset_fixture();
    /* 左轮原始计数方向取反，两轮都从 5 向下回绕时，报告的方向相反。 */
    htim3.Instance->CNT = 5U;
    htim2.Instance->CNT = 5U;
    assert(motor_speed_init());
    htim3.Instance->CNT = 65530U;
    htim2.Instance->CNT = 65530U;
    motor_speed_update(10U);
    motor_feedback_t left_feedback;
    motor_feedback_t right_feedback;
    assert(motor_speed_get_feedback(MOTOR_LEFT, &left_feedback));
    assert(motor_speed_get_feedback(MOTOR_RIGHT, &right_feedback));
    assert(left_feedback.delta_counts == 11);
    assert(right_feedback.delta_counts == -11);
    assert(left_feedback.total_counts == 11);
    float expected_speed_mm_s = 11.0f * MOTOR_DISTANCE_PER_COUNT_MM / 0.01f;
    assert_close(left_feedback.raw_speed_mm_s, expected_speed_mm_s);
    assert_close(left_feedback.filtered_speed_mm_s, 0.3f * expected_speed_mm_s);

    /* 重复查询反馈不得消费计数；dt=0 也不得丢掉尚未采样的增量。 */
    htim3.Instance->CNT = 65528U;
    assert(motor_speed_get_feedback(MOTOR_LEFT, &left_feedback));
    motor_speed_update(0U);
    assert(motor_speed_get_feedback(MOTOR_LEFT, &left_feedback));
    assert(left_feedback.total_counts == 11);
    motor_speed_update(20U);
    assert(motor_speed_get_feedback(MOTOR_LEFT, &left_feedback));
    assert(left_feedback.delta_counts == 2);
    assert(left_feedback.total_counts == 13);
    assert(!motor_speed_get_feedback(MOTOR_COUNT, &left_feedback));
    assert(!motor_speed_get_feedback(MOTOR_LEFT, NULL));
    assert(bsp_encoder_read_delta((motor_id_t)-1) == 0);

    htim3.Instance->CNT = 100U;
    motor_speed_reset();
    assert_close(motor_speed_get_distance_mm(MOTOR_LEFT), 0.0f);
    motor_speed_update(10U);
    assert(motor_speed_get_feedback(MOTOR_LEFT, &left_feedback));
    assert(left_feedback.total_counts == 0);
    assert_close(left_feedback.filtered_speed_mm_s, 0.0f);
    htim3.Instance->CNT = 1640U; /* 左轮原始计数增加 1540，反馈为反向一圈。 */
    motor_speed_update(1000U);
    assert_close(motor_speed_get_distance_mm(MOTOR_LEFT),
                 -2.0f * 3.14159265f * MOTOR_WHEEL_RADIUS_MM);
}

/** test_chassis_motion：验证斜坡、差速、等比限幅与两种停止行为。参数：无。返回：无。 */
static void test_chassis_motion(void) {
    reset_fixture();
    chassis_update(10U);
    assert_stopped();
    chassis_set_velocity(600.0f, 0.0f);
    chassis_update(0U);
    assert_stopped();
    chassis_update(10U); /* 指令从 0 增到 60mm/s，输出 343 千分比，CCR=34。 */
    assert(htim4.Instance->compare[0] == 34U);
    assert(htim4.Instance->compare[2] == 34U);
    chassis_update(10U);
    assert(htim4.Instance->compare[0] == 42U);
    chassis_request_stop();
    chassis_update(10U);
    assert(htim4.Instance->compare[0] == 34U);
    chassis_update(10U);
    assert_stopped();

    chassis_set_velocity(300.0f, 0.0f);
    chassis_update(100U);
    assert(htim4.Instance->compare[0] == 64U);
    chassis_stop();
    assert_stopped();
    chassis_update(100U);
    assert_stopped();

    /* 角速度 2rad/s、轮距 234mm，左右轮目标分别为 -234 和 +234mm/s。 */
    chassis_set_velocity(0.0f, 2.0f);
    chassis_update(100U);
    assert(htim4.Instance->compare[0] == 0U);
    assert(htim4.Instance->compare[1] == 56U);
    assert(htim4.Instance->compare[2] == 56U);
    assert(htim4.Instance->compare[3] == 0U);

    chassis_stop();
    chassis_set_wheel_speeds(1200.0f, 600.0f);
    chassis_update(100U);
    assert(htim4.Instance->compare[0] == 100U);
    assert(htim4.Instance->compare[2] == 64U);
    chassis_set_velocity(NAN, 0.0f);
    assert_stopped();
    chassis_update(100U);
    assert_stopped();
    chassis_set_wheel_speeds(INFINITY, 100.0f);
    chassis_update(10U);
    assert_stopped();
}

/** test_controller_and_initialization：验证开环接口边界及启动失败零输出。参数：无。返回：无。 */
static void test_controller_and_initialization(void) {
    reset_fixture();
    assert(app_motor_control_step(MOTOR_LEFT, 300.0f, 0.0f, 0.01f) == 635);
    assert(app_motor_control_step(MOTOR_LEFT, -300.0f, 100.0f, 0.01f) == -635);
    assert(app_motor_control_step(MOTOR_LEFT, 0.0f, 100.0f, 0.01f) == 0);
    assert(app_motor_control_step(MOTOR_LEFT, 1000.0f, 0.0f, 0.01f) == 1000);
    assert(app_motor_control_step(MOTOR_LEFT, 300.0f, 0.0f, 0.0f) == 0);
    assert(app_motor_control_step(MOTOR_LEFT, NAN, 0.0f, 0.01f) == 0);
    assert(app_motor_control_step(MOTOR_COUNT, 300.0f, 0.0f, 0.01f) == 0);

    chassis_set_velocity(300.0f, 0.0f);
    chassis_update(100U);
    fail_encoder_start = true;
    assert(!chassis_init());
    assert_stopped();
    chassis_set_velocity(300.0f, 0.0f);
    chassis_update(100U);
    assert_stopped();

    fail_encoder_start = false;
    fail_pwm_start = true;
    assert(!chassis_init());
    assert_stopped();
    fail_pwm_start = false;
    htim3.Instance->ARR = 999U;
    assert(!chassis_init());
    assert_stopped();
    htim3.Instance->ARR = 65535U;
    assert(chassis_init());
    chassis_update(100U);
    assert_stopped();
}

/** main：执行主机端回归测试。参数：无。返回：成功为 0，断言失败终止。 */
int main(void) {
    test_pwm_scaling();
    test_encoder_feedback();
    test_chassis_motion();
    test_controller_and_initialization();
    puts("PASS: PWM scaling, encoder feedback, differential drive, ramp, stops and initialization");
    return 0;
}
