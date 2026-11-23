#include "app_line_follow.h"
#include "bsp_line_sensor.h"
#include "chassis.h"
#include "line_follow_config.h"
#include "main.h"
#include "motor_speed.h"
#include "tim.h"
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

/* timer_registers / test_gpio_b：电机定时器和五路输入的测试寄存器。 */
static TIM_TypeDef timer_registers[3];
TIM_HandleTypeDef htim2 = {&timer_registers[0]};
TIM_HandleTypeDef htim3 = {&timer_registers[1]};
TIM_HandleTypeDef htim4 = {&timer_registers[2]};
GPIO_TypeDef test_gpio_b;
static const uint16_t output_pins[LINE_SENSOR_COUNT] = {LINE_OUT1_Pin, LINE_OUT2_Pin, LINE_OUT3_Pin,
                                                        LINE_OUT4_Pin, LINE_OUT5_Pin};

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

/** assert_close：比较浮点结果。actual/expected：实测和期望值。返回：无。 */
static void assert_close(float actual, float expected) {
    assert(fabsf(actual - expected) < 0.001f);
}

/** get_status：取得循迹缓存副本。参数：无。返回：当前状态。 */
static line_follow_status_t get_status(void) {
    line_follow_status_t status;
    assert(app_line_follow_get_status(&status));
    return status;
}

/**
 * set_line_mask：设置物理左到右的模拟黑线分布。
 * line_mask：bit0 最左、bit4 最右，1 为黑线。返回：无。
 * 按编译配置生成模块原始电平，测试同时覆盖低有效和高有效、正序和反序安装。
 */
static void set_line_mask(uint8_t line_mask) {
    test_gpio_b.IDR = 0U;
    for (uint8_t output_index = 0U; output_index < LINE_SENSOR_COUNT; ++output_index) {
        uint8_t physical_index =
            LINE_FOLLOW_REVERSE_SENSOR_ORDER ? LINE_SENSOR_COUNT - 1U - output_index : output_index;
        bool on_line = (line_mask & (1U << physical_index)) != 0U;
        bool high_level = LINE_FOLLOW_ACTIVE_LOW ? !on_line : on_line;
        if (high_level) {
            test_gpio_b.IDR |= output_pins[output_index];
        }
    }
}

/** reset_fixture：初始化实际底盘和循迹模块，默认模拟中间一只探头压线。返回：无。 */
static void reset_fixture(void) {
    for (unsigned int index = 0U; index < 3U; ++index) {
        timer_registers[index] = (TIM_TypeDef){0};
    }
    htim2.Instance->ARR = 65535U;
    htim3.Instance->ARR = 65535U;
    htim4.Instance->ARR = 99U;
    assert(chassis_init());
    app_line_follow_init();
    set_line_mask(0x04U);
}

/** control_tick：按实际主循环顺序执行一次。dt_ms：更新间隔，毫秒。返回：无。 */
static void control_tick(uint32_t dt_ms) {
    app_line_follow_update(dt_ms);
    chassis_update(dt_ms);
}

/** confirm_line：连续采样到有效线型稳定。参数：无。返回：无。 */
static void confirm_line(void) {
    for (unsigned int sample_index = 0U; sample_index < LINE_FOLLOW_STABLE_SAMPLES;
         ++sample_index) {
        control_tick(10U);
    }
}

/** assert_stopped：检查四路比较值及后续更新均保持零。参数：无。返回：无。 */
static void assert_stopped(void) {
    for (unsigned int channel = 0U; channel < 4U; ++channel) {
        assert(htim4.Instance->compare[channel] == 0U);
    }
}

/** test_gpio_mapping：逐个验证 PB12～PB15、PB3 对应原始 bit0～bit4。返回：无。 */
static void test_gpio_mapping(void) {
    for (uint8_t output_index = 0U; output_index < LINE_SENSOR_COUNT; ++output_index) {
        test_gpio_b.IDR = output_pins[output_index] | (1U << 6);
        assert(bsp_line_sensor_read_raw() == (uint8_t)(1U << output_index));
    }
    test_gpio_b.IDR = 0U;
    assert(bsp_line_sensor_read_raw() == 0U);
}

/** test_start_stop_and_manual_mode：验证默认停住、明确启动、退出后手动控制。返回：无。 */
static void test_start_stop_and_manual_mode(void) {
    reset_fixture();
    confirm_line();
    assert(!get_status().enabled);
    assert(get_status().state == LINE_FOLLOW_DISABLED);
    assert(get_status().line_mask == 0x04U);
    assert_stopped();
    assert(!app_line_follow_get_status(NULL));

    chassis_set_velocity(200.0f, 0.0f);
    control_tick(100U); /* 禁用时采样不能抢占外部手动指令。 */
    assert(htim4.Instance->compare[0] > 0U);
    app_line_follow_start();
    assert_stopped();
    assert(get_status().state == LINE_FOLLOW_ACQUIRING);

    for (unsigned int sample_index = 1U; sample_index < LINE_FOLLOW_STABLE_SAMPLES;
         ++sample_index) {
        control_tick(10U);
        assert(get_status().state == LINE_FOLLOW_ACQUIRING);
        assert_stopped();
    }
    control_tick(10U);
    assert(get_status().state == LINE_FOLLOW_TRACKING);
    assert_close(get_status().linear_speed_mm_s, 120.0f);
    assert_close(get_status().angular_speed_rad_s, 0.0f);
    assert(htim4.Instance->compare[0] == htim4.Instance->compare[3]);
    assert(htim4.Instance->compare[0] > 0U);

    app_line_follow_stop();
    assert(!get_status().enabled);
    assert_stopped();
    confirm_line();
    assert_stopped();
    chassis_set_velocity(200.0f, 0.0f);
    control_tick(100U);
    assert(htim4.Instance->compare[0] > 0U);
}

/** test_all_patterns：枚举五路全部 32 种分布，核对位置和无效线型分类。返回：无。 */
static void test_all_patterns(void) {
    /* 按物理左到右编号的已知正常线型与人工列出的期望位置偏差。 */
    static const uint8_t normal_masks[] = {1, 2, 4, 8, 16, 3, 6, 12, 24};
    static const float expected_errors[] = {2.0f, 1.0f, 0.0f,  -1.0f, -2.0f,
                                            1.5f, 0.5f, -0.5f, -1.5f};
    static const uint8_t wide_masks[] = {7, 14, 28, 15, 30, 31};
    static const uint8_t ambiguous_masks[] = {5,  9,  10, 11, 13, 17, 18, 19,
                                              20, 21, 22, 23, 25, 26, 27, 29};
    bool covered[32] = {false};
    reset_fixture();
    for (unsigned int pattern_index = 0U; pattern_index < sizeof(normal_masks); ++pattern_index) {
        uint8_t line_mask = normal_masks[pattern_index];
        covered[line_mask] = true;
        app_line_follow_start();
        set_line_mask(line_mask);
        confirm_line();
        control_tick(100U); /* 让现有轮速斜坡到达目标，再检查左右输出大小关系。 */
        line_follow_status_t status = get_status();
        assert(status.state == LINE_FOLLOW_TRACKING);
        assert(status.line_mask == line_mask);
        assert_close(status.position_error, expected_errors[pattern_index]);
        assert(fabsf(status.angular_speed_rad_s) <= LINE_FOLLOW_MAX_ANGULAR_SPEED_RAD_S);
        if (expected_errors[pattern_index] > 0.0f) {
            assert(status.angular_speed_rad_s > 0.0f);
            assert(htim4.Instance->compare[3] > htim4.Instance->compare[0]);
        } else if (expected_errors[pattern_index] < 0.0f) {
            assert(status.angular_speed_rad_s < 0.0f);
            assert(htim4.Instance->compare[0] > htim4.Instance->compare[3]);
        } else {
            assert_close(status.angular_speed_rad_s, 0.0f);
        }
    }

    for (unsigned int group_index = 0U; group_index < 2U; ++group_index) {
        const uint8_t *masks = group_index == 0U ? wide_masks : ambiguous_masks;
        size_t mask_count = group_index == 0U ? sizeof(wide_masks) : sizeof(ambiguous_masks);
        line_follow_state_t expected_state =
            group_index == 0U ? LINE_FOLLOW_WIDE_LINE : LINE_FOLLOW_AMBIGUOUS;
        for (size_t pattern_index = 0U; pattern_index < mask_count; ++pattern_index) {
            reset_fixture();
            app_line_follow_start();
            confirm_line(); /* 先确保确实在运动，再验证异常图案会停车。 */
            uint8_t line_mask = masks[pattern_index];
            covered[line_mask] = true;
            set_line_mask(line_mask);
            control_tick(10U);
            assert(get_status().state == expected_state);
            assert(get_status().enabled);
            assert_stopped();
            chassis_update(100U);
            assert_stopped(); /* 确保最终目标和斜坡也已清零，不会下周期重新启动。 */
        }
    }
    covered[0] = true;
    set_line_mask(0U);
    control_tick(10U);
    assert(get_status().state == LINE_FOLLOW_LOST);
    assert_stopped();
    for (unsigned int mask = 0U; mask < 32U; ++mask) {
        assert(covered[mask]);
    }
}

/** test_debounce_reacquisition_and_sampling：验证抖动、重新捕获和采样唯一性。返回：无。 */
static void test_debounce_reacquisition_and_sampling(void) {
    reset_fixture();
    app_line_follow_start();
    confirm_line();
    assert_close(get_status().angular_speed_rad_s, 0.0f);
    set_line_mask(0x02U);
    control_tick(10U);
    assert_close(get_status().angular_speed_rad_s, 0.0f); /* 短暂有效线型不立刻转向。 */
    set_line_mask(0x04U);
    confirm_line();
    set_line_mask(0x02U);
    confirm_line();
    assert_close(get_status().angular_speed_rad_s, 0.8f);
    line_follow_status_t copy = get_status();
    copy.enabled = false;
    assert(copy.enabled != get_status().enabled); /* 修改副本不改变内部状态。 */

    /* 查询及 dt=0 不采样，也不推进连续样本计数。 */
    uint8_t previous_raw = get_status().raw_levels;
    set_line_mask(0U);
    control_tick(0U);
    assert(get_status().raw_levels == previous_raw);
    control_tick(10U);
    assert(get_status().state == LINE_FOLLOW_LOST);
    assert_stopped();
    set_line_mask(0x04U);
    control_tick(10U);
    assert(get_status().state == LINE_FOLLOW_ACQUIRING);
    assert_stopped();
    confirm_line();
    assert(get_status().state == LINE_FOLLOW_TRACKING);

    htim3.Instance->CNT += 10U;
    app_line_follow_update(10U);
    motor_feedback_t feedback;
    assert(motor_speed_get_feedback(MOTOR_LEFT, &feedback));
    assert(feedback.total_counts == 0); /* 循迹模块没有抢先消费编码器计数。 */
    chassis_update(10U);
    assert(motor_speed_get_feedback(MOTOR_LEFT, &feedback));
    assert(feedback.delta_counts == 10);
    assert(feedback.total_counts == 10);
}

/** main：运行真实循迹和底盘源码的主机回归。参数：无。返回：成功为 0。 */
int main(void) {
    test_gpio_mapping();
    test_start_stop_and_manual_mode();
    test_all_patterns();
    test_debounce_reacquisition_and_sampling();
    puts("PASS: line GPIO mapping, all 32 patterns, steering, stops, debounce and manual mode");
    return 0;
}
