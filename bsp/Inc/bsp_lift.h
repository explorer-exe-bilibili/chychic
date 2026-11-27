#ifndef BSP_LIFT_H
#define BSP_LIFT_H

#include <stdbool.h>
#include <stdint.h>

/** 启动 TIM1 的两路 PWM，初始占空比为零。返回两路是否都已启动。 */
bool bsp_lift_init(void);

/** 设置电机输出，pwm_permille 为有符号千分比，正数上升，负数下降。 */
void bsp_lift_set_pwm(int16_t pwm_permille);

/** 在编码器 A 相上升沿中断中调用，根据 B 相电平增减计数。 */
void bsp_lift_encoder_on_a_edge(void);

/** 返回从上电开始的编码器计数，按 32 位无符号整数回绕。 */
uint32_t bsp_lift_encoder_count(void);

#endif
