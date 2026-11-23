#ifndef BSP_MOTOR_ENCODER_H
#define BSP_MOTOR_ENCODER_H

#include "motor_types.h"
#include <stdbool.h>
#include <stdint.h>

/**两个编码器启动成功为 true，否则为 false。*/
bool bsp_encoder_init(void);

/**
 * motor_id：左右轮编号。返回：计数增量，无效编号返回 0。
 * 仅供 motor_speed 模块采样；
 * 要求 ARR=65535
 */
int32_t bsp_encoder_read_delta(motor_id_t motor_id);

#endif
