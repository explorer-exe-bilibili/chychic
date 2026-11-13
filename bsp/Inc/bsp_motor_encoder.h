//
// Created by moonrise on 2026/11/12.
//

#ifndef CHYCHIC_PROJECT_BSP_MOTOR_ENCODER_H
#define CHYCHIC_PROJECT_BSP_MOTOR_ENCODER_H
#include <stdint.h>

#define ENC_LEFT   0
#define ENC_RIGHT  1

void    encoder_init(void);
int32_t encoder_get_delta(uint8_t id);   /* 自上次调用以来的增量（带符号） */
int32_t encoder_get_total(uint8_t id);   /* 上电至今累计位置（带符号） */
void    encoder_reset_total(uint8_t id);
void    encoder_set_sign(uint8_t id, int8_t sign);  /* +1 / -1，用来纠正接反的轮子 */

#endif //CHYCHIC_PROJECT_BSP_MOTOR_ENCODER_H
