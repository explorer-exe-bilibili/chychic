#include "app_line_follow.h"
#include "app_motor.h"
#include "chassis.h"
#include "line_follow_config.h"
#include "main.h"
#include "pid.h"
#include "tim.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* timer_registers / test_gpio_b：用于验证实际控制路径的硬件替身。 */
static TIM_TypeDef timer_registers[3];
TIM_HandleTypeDef htim2 = {&timer_registers[0]};
TIM_HandleTypeDef htim3 = {&timer_registers[1]};
TIM_HandleTypeDef htim4 = {&timer_registers[2]};
GPIO_TypeDef test_gpio_b;

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel) {
    (void)timer;
    (void)channel;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channel) {
    (void)timer;
    (void)channel;
    return HAL_OK;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *gpio_port, uint16_t gpio_pin) {
    return (gpio_port->IDR & gpio_pin) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/** assert_close：比较实际与期望的浮点输出。返回：无。 */
static void assert_close(float actual, float expected) {
    assert(fabsf(actual - expected) < 0.001f);
}

/** reset_fixture：清空替身寄存器并初始化双轮和循迹。参数：无。返回：无。 */
static void reset_fixture(void) {
    timer_registers[0] = (TIM_TypeDef){.ARR = 65535U};
    timer_registers[1] = (TIM_TypeDef){.ARR = 65535U};
    timer_registers[2] = (TIM_TypeDef){.ARR = 99U};
    assert(chassis_init());
    app_line_follow_init();
}

/** test_pid_terms：覆盖实际周期、积分饱和与溢出复位。参数：无。返回：无。 */
static void test_pid_terms(void) {
    pid_state_t state = {0};
    const pid_gains_t gains = {.kp = 2.0f, .ki = 3.0f, .kd = 0.5f};
    float output;
    assert(pid_step(&state, &gains, 2.0f, 0.1f, 1.0f, -10.0f, 10.0f, &output));
    assert_close(output, 5.6f);
    assert(pid_step(&state, &gains, 1.0f, 0.2f, 1.0f, -10.0f, 10.0f, &output));
    assert_close(output, 1.7f);

    assert(!pid_step(&state, &gains, INFINITY, 0.1f, 0.0f, -10.0f, 10.0f, &output));
    assert_close(output, 0.0f);
    assert(pid_step(&state, &gains, 1.0f, 0.2f, 1.0f, -10.0f, 10.0f, &output));
    assert_close(output, 3.6f);
    assert(!pid_step(&state, &gains, 3.0e38f, 1.0f, 0.0f, -10.0f, 10.0f, &output));
    assert_close(output, 0.0f);

    const pid_gains_t integral_only = {.ki = 1.0f};
    assert(pid_step(&state, &integral_only, 4.0f, 1.0f, 0.0f, -5.0f, 5.0f, &output));
    assert_close(output, 4.0f);
    assert(pid_step(&state, &integral_only, 6.0f, 1.0f, 0.0f, -5.0f, 5.0f, &output));
    assert_close(output, 5.0f);
    assert(pid_step(&state, &integral_only, 6.0f, 1.0f, 0.0f, -5.0f, 5.0f, &output));
    assert_close(output, 5.0f);
    assert(pid_step(&state, &integral_only, -1.0f, 1.0f, 10.0f, -5.0f, 5.0f, &output));
    assert_close(output, 5.0f); /* 前馈仍使输出饱和，但积分已回退。 */
    assert(pid_step(&state, &integral_only, 0.0f, 1.0f, 0.0f, -5.0f, 5.0f, &output));
    assert_close(output, 4.0f);
}

/** test_wheel_pid：验证分轮增益、方向限幅及复位。参数：无。返回：无。 */
static void test_wheel_pid(void) {
    reset_fixture();
    assert(app_motor_control_step(MOTOR_LEFT, 100.0f, 50.0f, 0.1f) == 512);
    assert(app_motor_control_step(MOTOR_RIGHT, 100.0f, 50.0f, 0.1f) == 452);
    assert(app_motor_control_step(MOTOR_LEFT, 100.0f, 50.0f, 0.1f) == 532);
    assert(app_motor_control_step(MOTOR_LEFT, 100.0f, 1000.0f, 0.1f) == 0);
    assert(app_motor_control_step(MOTOR_LEFT, -100.0f, -100.0f, 0.1f) == -392);
    assert(app_motor_control_step(MOTOR_LEFT, -100.0f, -1000.0f, 0.1f) == 0);
    assert(app_motor_control_step(MOTOR_LEFT, 100.0f, -1000.0f, 0.1f) == 1000);
    assert(app_motor_control_step(MOTOR_LEFT, -100.0f, 1000.0f, 0.1f) == -1000);
    assert(app_motor_control_step(MOTOR_RIGHT, 0.5f, 50.0f, 0.1f) == 0);
    assert(app_motor_control_step(MOTOR_RIGHT, 100.0f, 100.0f, 0.1f) == 392);

    assert(app_motor_control_step(MOTOR_LEFT, 100.0f, 50.0f, 0.1f) == 512);
    assert(app_motor_control_step(MOTOR_RIGHT, 100.0f, 50.0f, 0.1f) == 452);
    chassis_stop();
    assert(app_motor_control_step(MOTOR_LEFT, 100.0f, 100.0f, 0.1f) == 392);
    assert(app_motor_control_step(MOTOR_RIGHT, 100.0f, 100.0f, 0.1f) == 392);
}

/** test_encoder_to_pwm：验证编码器变化只修正对应轮的输出。参数：无。返回：无。 */
static void test_encoder_to_pwm(void) {
    reset_fixture();
    chassis_set_wheel_speeds(100.0f, 100.0f);
    chassis_update(100U);
    uint32_t left_without_feedback = htim4.Instance->compare[0];
    uint32_t right_without_feedback = htim4.Instance->compare[2];
    assert(left_without_feedback == 63U);
    assert(right_without_feedback == 51U);

    reset_fixture();
    chassis_set_wheel_speeds(100.0f, 100.0f);
    htim3.Instance->CNT = 65516U; /* 左轮有正向增量，右轮保持静止。 */
    chassis_update(100U);
    assert(htim4.Instance->compare[0] < left_without_feedback);
    assert(htim4.Instance->compare[2] == right_without_feedback);
}

/** steering_tick：更新一次实际控制循环并读取循迹状态。dt_ms：周期毫秒数。 */
static line_follow_status_t steering_tick(uint32_t dt_ms) {
    app_line_follow_update(dt_ms);
    chassis_update(dt_ms);
    line_follow_status_t status;
    assert(app_line_follow_get_status(&status));
    return status;
}

/** test_steering_pid：验证去抖间隔、角速度限幅和丢线后的重新捕获。参数：无。返回：无。 */
static void test_steering_pid(void) {
    reset_fixture();
    const uint32_t sensor_pins =
        LINE_OUT1_Pin | LINE_OUT2_Pin | LINE_OUT3_Pin | LINE_OUT4_Pin | LINE_OUT5_Pin;
    test_gpio_b.IDR = sensor_pins & ~LINE_OUT2_Pin;
    app_line_follow_start();
    for (unsigned int sample = 1U; sample < LINE_FOLLOW_STABLE_SAMPLES; ++sample) {
        assert(steering_tick(10U).state == LINE_FOLLOW_ACQUIRING);
    }
    assert_close(steering_tick(10U).angular_speed_rad_s, 0.21f);
    assert_close(steering_tick(20U).angular_speed_rad_s, 0.23f);

    test_gpio_b.IDR = sensor_pins & ~(LINE_OUT2_Pin | LINE_OUT3_Pin);
    assert_close(steering_tick(5U).angular_speed_rad_s, 0.23f);
    assert_close(steering_tick(15U).angular_speed_rad_s, 0.23f);
    assert_close(steering_tick(20U).angular_speed_rad_s, -0.60f); /* 间隔共 40ms。 */

    test_gpio_b.IDR = sensor_pins & ~LINE_OUT1_Pin;
    for (unsigned int sample = 1U; sample < LINE_FOLLOW_STABLE_SAMPLES; ++sample) {
        (void)steering_tick(10U);
    }
    assert_close(steering_tick(10U).angular_speed_rad_s, LINE_FOLLOW_MAX_ANGULAR_SPEED_RAD_S);

    test_gpio_b.IDR = sensor_pins;
    assert(steering_tick(10U).state == LINE_FOLLOW_LOST);
    for (unsigned int channel = 0; channel < 4U; ++channel) {
        assert(htim4.Instance->compare[channel] == 0U);
    }
    test_gpio_b.IDR = sensor_pins & ~LINE_OUT4_Pin;
    for (unsigned int sample = 1U; sample < LINE_FOLLOW_STABLE_SAMPLES; ++sample) {
        assert(steering_tick(10U).state == LINE_FOLLOW_ACQUIRING);
    }
    assert_close(steering_tick(10U).angular_speed_rad_s, -0.21f);
}

/** 验证在线调参只修改目标轮，清除该轮历史，并能上报斜坡后的双轮目标。 */
static void test_online_wheel_pid(void) {
    reset_fixture();
    assert(app_motor_control_step(MOTOR_LEFT, 100.0f, 20.0f, 0.1f) == 584);

    const pid_gains_t gains = {.kp = 1.0f, .ki = 2.0f, .kd = 1.0f};
    pid_gains_t right_default;
    assert(app_motor_get_speed_pid(MOTOR_RIGHT, &right_default));
    assert(app_motor_set_speed_pid(MOTOR_LEFT, &gains));
    pid_gains_t active;
    assert(app_motor_get_speed_pid(MOTOR_LEFT, &active));
    assert_close(active.kp, 1.0f);
    assert_close(active.ki, 2.0f);
    assert_close(active.kd, 1.0f);
    assert(app_motor_get_speed_pid(MOTOR_RIGHT, &active));
    assert_close(active.kp, right_default.kp);
    assert_close(active.ki, right_default.ki);
    assert_close(active.kd, right_default.kd);

    assert(app_motor_control_step(MOTOR_LEFT, 100.0f, 50.0f, 0.1f) == 452);
    assert(app_motor_set_speed_pid(MOTOR_LEFT, &gains));
    assert(app_motor_control_step(MOTOR_LEFT, 100.0f, 50.0f, 0.1f) == 462);
    const pid_gains_t invalid = {.kp = -1.0f, .ki = NAN, .kd = INFINITY};
    assert(!app_motor_set_speed_pid(MOTOR_LEFT, &invalid));
    assert(app_motor_control_step(MOTOR_LEFT, 100.0f, 50.0f, 0.1f) == 472);

    float left_mm_s;
    float right_mm_s;
    chassis_set_wheel_speeds(100.0f, 40.0f);
    chassis_update(10U);
    chassis_get_wheel_targets(&left_mm_s, &right_mm_s);
    assert_close(left_mm_s, 60.0f);
    assert_close(right_mm_s, 40.0f);
    chassis_stop();
    chassis_get_wheel_targets(&left_mm_s, &right_mm_s);
    assert_close(left_mm_s, 0.0f);
    assert_close(right_mm_s, 0.0f);
}

/** main：运行已配置增益的控制测试。参数：无。返回：成功为 0。 */
int main(void) {
    test_pid_terms();
    test_wheel_pid();
    test_encoder_to_pwm();
    test_steering_pid();
    test_online_wheel_pid();
    puts("PASS: PID terms, anti-windup, wheel feedback, and line steering");
    return 0;
}
