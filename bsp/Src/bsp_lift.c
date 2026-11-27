#include "bsp_lift.h"
#include "lift_config.h"
#include "tim.h"

/* TIM1 两个通道分别接驱动板的 IN1/IN2。 */
#if LIFT_UP_ON_CH1
#define LIFT_UP_CHANNEL TIM_CHANNEL_1
#define LIFT_DOWN_CHANNEL TIM_CHANNEL_4
#else
#define LIFT_UP_CHANNEL TIM_CHANNEL_4
#define LIFT_DOWN_CHANNEL TIM_CHANNEL_1
#endif

/* 只在 EXTI 中断里写入，主循环读取对齐的 32 位值。 */
static volatile uint32_t encoder_count;

bool bsp_lift_init(void) {
    bsp_lift_set_pwm(0);
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
        return false;
    }
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
        (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        return false;
    }
    return true;
}

void bsp_lift_set_pwm(int16_t pwm_permille) {
    int32_t duty_permille = pwm_permille;
    if (duty_permille > 1000) {
        duty_permille = 1000;
    } else if (duty_permille < -1000) {
        duty_permille = -1000;
    }

    uint32_t magnitude = (uint32_t)(duty_permille < 0 ? -duty_permille : duty_permille);
    uint32_t period_counts = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
    uint32_t compare = (period_counts * magnitude + 500U) / 1000U;

    /* 暂停更新事件，两路预装载值在同一个 PWM 周期切换。 */
    uint32_t updates_disabled = READ_BIT(htim1.Instance->CR1, TIM_CR1_UDIS);
    SET_BIT(htim1.Instance->CR1, TIM_CR1_UDIS);
    __HAL_TIM_SET_COMPARE(&htim1, LIFT_UP_CHANNEL, duty_permille > 0 ? compare : 0U);
    __HAL_TIM_SET_COMPARE(&htim1, LIFT_DOWN_CHANNEL, duty_permille < 0 ? compare : 0U);
    if (updates_disabled == 0U) {
        CLEAR_BIT(htim1.Instance->CR1, TIM_CR1_UDIS);
    }
}

void bsp_lift_encoder_on_a_edge(void) {
    GPIO_PinState b_level = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
    if (b_level == (LIFT_ENCODER_UP_B_HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET)) {
        encoder_count++;
    } else {
        encoder_count--;
    }
}

uint32_t bsp_lift_encoder_count(void) {
    return encoder_count;
}
