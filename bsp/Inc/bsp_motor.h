//
// Created by moonrise on 2026/11/12.
//

#ifndef CHYCHIC_PROJECT_BSP_MOTOR_H
#define CHYCHIC_PROJECT_BSP_MOTOR_H
#include "tim.h"
extern int32_t pulse_left;
extern int32_t pulse_right;
void bsp_motor_init(void);
void left_motor_set_speed(int32_t speed);
void right_motor_set_speed(int32_t speed);
#endif //CHYCHIC_PROJECT_BSP_MOTOR_H
