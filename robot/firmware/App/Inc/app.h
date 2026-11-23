/**
 * @file    app.h
 * @brief   应用层入口：把协议、电机、看门狗串起来
 *
 * 接入 CubeMX 工程只需要三步（详见 firmware/README.md）：
 *   1. main() 里 USER CODE 区调 app_init()
 *   2. HAL_UART_RxCpltCallback 里调 app_on_uart_rx(byte) 并重新武装接收
 *   3. 1ms 定时器中断（或 SysTick）里调 app_tick_1khz()
 */
#ifndef APP_H
#define APP_H

#include <stdint.h>

/** HAL_UART_RxCpltCallback 里要把这个字节交给 app_on_uart_rx() */
extern uint8_t app_rx_byte;

void app_init(void);

/** UART 每收到一个字节调用；内部会自动重新武装中断接收 */
void app_on_uart_rx(uint8_t byte);

/** 1kHz 调用：测速、闭环、看门狗、100Hz 上报 */
void app_tick_1khz(void);

#endif /* APP_H */
