//
// Created by moonrise on 2026/11/12.
//

#include "bsp_motor_encoder.h"
#include "tim.h"     /* htim2 / htim3 */

typedef struct {
    TIM_HandleTypeDef *htim;
    uint16_t last_cnt;
    int32_t  total;
    int32_t  delta;
    int8_t   sign;          /* 右轮接反了就设 -1，不用拆线 */
} encoder_t;

static encoder_t enc[2] = {
    { &htim3, 0, 0, 0, +1 },   /* 左轮 PA6/PA7 → TIM3 */
    { &htim2, 0, 0, 0, +1 },   /* 右轮 PA0/PA1 → TIM2 */
};

void encoder_init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    enc[ENC_LEFT ].last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(enc[ENC_LEFT ].htim);
    enc[ENC_RIGHT].last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(enc[ENC_RIGHT].htim);
}

int32_t encoder_get_delta(uint8_t id)
{
    encoder_t *e = &enc[id];

    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(e->htim);
    int32_t  d   = (int16_t)(now - e->last_cnt);

    e->last_cnt = now;
    e->delta    = d * e->sign;
    e->total   += e->delta;
    return e->delta;
}

int32_t encoder_get_total(uint8_t id) { return enc[id].total; }
void    encoder_reset_total(uint8_t id) { enc[id].total = 0; }
void    encoder_set_sign(uint8_t id, int8_t sign) { enc[id].sign = (sign >= 0) ? +1 : -1; }
/*以上所有的id都表示左右轮，id=0表示左轮，id=1表示右轮*/