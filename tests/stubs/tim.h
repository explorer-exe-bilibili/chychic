#ifndef TEST_TIM_H
#define TEST_TIM_H

#include <stdint.h>

/* 主机测试只模拟寄存器读写，不模拟 PWM 波形、预装载时序或真实电机。 */
typedef struct {
    uint32_t CR1;
    uint32_t ARR;
    uint32_t CNT;
    uint32_t compare[4];
} TIM_TypeDef;

typedef struct {
    TIM_TypeDef *Instance;
} TIM_HandleTypeDef;

typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 1U
#define TIM_CHANNEL_3 2U
#define TIM_CHANNEL_4 3U
#define TIM_CHANNEL_ALL 15U
#define TIM_CR1_UDIS 2U
#define READ_BIT(reg, mask) ((reg) & (mask))
#define SET_BIT(reg, mask) ((reg) |= (mask))
#define CLEAR_BIT(reg, mask) ((reg) &= ~(mask))
#define __HAL_TIM_GET_COUNTER(timer) ((timer)->Instance->CNT)
#define __HAL_TIM_GET_AUTORELOAD(timer) ((timer)->Instance->ARR)
#define __HAL_TIM_SET_COMPARE(timer, channel, value) ((timer)->Instance->compare[channel] = (value))

/* HAL_TIM_PWM_Start/HAL_TIM_Encoder_Start：测试替身；参数为句柄和通道，返回模拟状态。 */
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channel);

#endif
