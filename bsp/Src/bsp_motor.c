//
// Created by moonrise on 2026/11/12.
//

#include "../Inc/bsp_motor.h"
int32_t pulse_left = 0;
int32_t pulse_right = 0;

void bsp_motor_init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
}

void left_motor_set_speed(int32_t speed)
{
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    if (speed >0)//正转
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, speed);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    }
    else if (speed < 0)//反转
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, -speed);

    }
    else//停止
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    }
}
void right_motor_set_speed(int32_t speed)
{
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    if (speed >0)//正转
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, speed);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    }
    else if (speed < 0)//反转
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, -speed);

    }
    else//停止
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    }
}
