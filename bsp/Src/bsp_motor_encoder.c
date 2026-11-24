#include "bsp_motor_encoder.h"
#include "tim.h"


typedef struct {
    TIM_HandleTypeDef *timer_handle; /* 编码器使用的定时器句柄。 */
    uint16_t previous_count;         /* 上次采样时的 16 位计数。 */
    int32_t direction_sign;          /* 反馈方向修正，与电机驱动方向独立标定。 */
} encoder_state_t;

static encoder_state_t encoder_states[MOTOR_COUNT] = {
    {&htim3, 0, -1}, /* 左轮：TIM3 CH1/CH2，PB4/PB5，部分重映射。 */
    {&htim2, 0, +1}, /* 右轮：TIM2 CH1/CH2，PA0/PA1。 */
};

bool bsp_encoder_init(void) {
    for (motor_id_t motor_id = MOTOR_LEFT; motor_id < MOTOR_COUNT; ++motor_id) {
        encoder_state_t *encoder_state = &encoder_states[motor_id];
        if (__HAL_TIM_GET_AUTORELOAD(encoder_state->timer_handle) != UINT16_MAX ||
            HAL_TIM_Encoder_Start(encoder_state->timer_handle, TIM_CHANNEL_ALL) != HAL_OK) {
            return false;
        }
        encoder_state->previous_count =
            (uint16_t)__HAL_TIM_GET_COUNTER(encoder_state->timer_handle);//记录当前计数器值，作为下次采样的基准
    }
    return true;
}
/**读取指定轮子的本次计数增量。 */
int32_t bsp_encoder_read_delta(motor_id_t motor_id) {
    if ((unsigned int)motor_id >= MOTOR_COUNT) {
        return 0;
    }
    encoder_state_t *encoder_state = &encoder_states[motor_id];
    uint16_t current_count = (uint16_t)__HAL_TIM_GET_COUNTER(encoder_state->timer_handle);//读取当前计数器值
    uint16_t wrapped_delta = (uint16_t)(current_count - encoder_state->previous_count);
    /* 处理反绕问题 */
    int32_t delta_counts =
        wrapped_delta < 32768U ? (int32_t)wrapped_delta : (int32_t)wrapped_delta - 65536;
    encoder_state->previous_count = current_count;//更新上次采样值
    return delta_counts * encoder_state->direction_sign;//修正方向后返回增量
}
