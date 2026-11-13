#include "bsp_motor.h"
#include "tim.h"

/** 记录各电机的 PWM 通道和车辆前进方向的接线极性。 */
typedef struct {
    uint32_t forward_channel; /* 正向输入对应的 TIM4 通道。 */
    uint32_t reverse_channel; /* 反向输入对应的 TIM4 通道。 */
    int32_t direction_sign;   /* 接线修正：+1 不翻转，-1 翻转。 */
} motor_pwm_binding_t;

/** 左右轮映射及右轮反向设置。 */
static const motor_pwm_binding_t motor_pwm_bindings[MOTOR_COUNT] = {
    {TIM_CHANNEL_1, TIM_CHANNEL_2, +1}, /* 左轮：PB6/PB7。 */
    {TIM_CHANNEL_3, TIM_CHANNEL_4, +1}, /* 右轮：PB8/PB9。 */
};

/**写入期间暂停更新事件，避免两个新值在不同 PWM 周期生效。 */
static void motor_write_compare(const motor_pwm_binding_t *binding, uint32_t forward_compare,
                                uint32_t reverse_compare) {
    uint32_t previous_update_disable = READ_BIT(htim4.Instance->CR1, TIM_CR1_UDIS);//0为允许更新事件，1为禁止更新事件
    SET_BIT(htim4.Instance->CR1, TIM_CR1_UDIS);//禁止更新事件
    __HAL_TIM_SET_COMPARE(&htim4, binding->forward_channel, forward_compare);
    __HAL_TIM_SET_COMPARE(&htim4, binding->reverse_channel, reverse_compare);
    if (previous_update_disable == 0U) {
        CLEAR_BIT(htim4.Instance->CR1, TIM_CR1_UDIS);
    }
}

bool bsp_motor_init(void) {
    bsp_motor_stop_all();
    for (motor_id_t motor_id = MOTOR_LEFT; motor_id < MOTOR_COUNT; ++motor_id) {
        const motor_pwm_binding_t *binding = &motor_pwm_bindings[motor_id];
        if (HAL_TIM_PWM_Start(&htim4, binding->forward_channel) != HAL_OK ||
            HAL_TIM_PWM_Start(&htim4, binding->reverse_channel) != HAL_OK) {
            bsp_motor_stop_all();
            return false;
        }
    }
    return true;
}
/**设置电机PWM占空比，正负表示方向，限幅到[-1000,1000]。*/
void bsp_motor_set_pwm(motor_id_t motor_id, int32_t pwm_permille)
{
    if ((unsigned int)motor_id >= MOTOR_COUNT) {
        return;
    }
    if (pwm_permille > MOTOR_PWM_MAX_PERMILLE) {
        pwm_permille = MOTOR_PWM_MAX_PERMILLE;
    } else if (pwm_permille < -MOTOR_PWM_MAX_PERMILLE) {
        pwm_permille = -MOTOR_PWM_MAX_PERMILLE;
    }

    const motor_pwm_binding_t *binding = &motor_pwm_bindings[motor_id];
    int32_t signed_pwm_permille = pwm_permille * binding->direction_sign;//只有方向
    uint32_t magnitude_permille =(uint32_t)(signed_pwm_permille < 0 ? -signed_pwm_permille : signed_pwm_permille);//绝对值
    uint32_t period_counts = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1U;//单个周期内的计数

    /* 计算CCR，四舍五入到最接近的整数。 */
    uint32_t compare_counts =
        (magnitude_permille * period_counts + MOTOR_PWM_MAX_PERMILLE / 2U) / MOTOR_PWM_MAX_PERMILLE;

    /* 防止 CCR 超出范围 ，其实用不到应该*/
    if (compare_counts > UINT16_MAX) {
        compare_counts = UINT16_MAX;
    }

    motor_write_compare(binding, signed_pwm_permille > 0 ? compare_counts : 0U,
                        signed_pwm_permille < 0 ? compare_counts : 0U);
}

void bsp_motor_stop_all(void) {
    bsp_motor_set_pwm(MOTOR_LEFT, 0);
    bsp_motor_set_pwm(MOTOR_RIGHT, 0);
}
