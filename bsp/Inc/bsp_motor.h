#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "motor_types.h"
#include <stdbool.h>
#include <stdint.h>

/** PWM 满量程：1000 表示 100%，与定时器 ARR 数值无关。 */
#define MOTOR_PWM_MAX_PERMILLE 1000

bool bsp_motor_init(void);

/**
 * bsp_motor_set_pwm：按车辆前进方向设置指定电机的带符号占空比。
 * motor_id：左右电机编号；pwm_permille：千分比，限幅到 [-1000, 1000]。
 */
void bsp_motor_set_pwm(motor_id_t motor_id, int32_t pwm_permille);

void bsp_motor_stop_all(void);

#endif
