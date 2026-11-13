//
// Created by moonrise on 2026/11/13.
//

#ifndef CHYCHIC_PROJECT_GETTICK_H
#define CHYCHIC_PROJECT_GETTICK_H
#include "tim.h"
/**
 * Get_tick：沿用现有计时包装接口，读取 HAL 系统毫秒计数。
 *
 * 参数：无。返回：上电后的毫秒计数，按 uint32_t 自然回绕。
 */
uint32_t Get_tick(void);

#endif // CHYCHIC_PROJECT_GETTICK_H
