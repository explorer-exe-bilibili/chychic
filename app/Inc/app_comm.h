/**
 * @file    app_comm.h
 * @brief   上下位机通信：接收解析、模式分发、遥测上报、链路看门狗
 *
 * 数据流
 *   USART1 RX --DMA循环--> 环形缓冲 --> proto_feed 逐字节解析
 *   --> 回调 on_vel / on_mode / on_estop --> chassis / app_line_follow
 *
 *   app_comm_update 每周期组一帧 FB_STATE --DMA--> USART1 TX
 */
#ifndef APP_COMM_H
#define APP_COMM_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/** 链路看门狗超时，毫秒。上位机以 20Hz 发 CMD_PING，留足余量。 */
#define APP_COMM_WATCHDOG_MS 300u

/** 遥测上报周期，毫秒。20Hz 对任务层足够，也不会挤占带宽。 */
#define APP_COMM_TELEMETRY_MS 50u

/**
 * app_comm_init：绑定串口、启动循环 DMA 接收、复位协议状态。
 * huart：已由 MX_USART1_UART_Init 初始化的串口句柄，不可为 NULL。
 * 返回：无。应在 MX_USART1_UART_Init 之后、主循环之前调用一次。
 */
void app_comm_init(UART_HandleTypeDef *huart);

/**
 * app_comm_poll：把 DMA 里新到的字节喂进解析器。
 * 参数：无。返回：无。
 * 必须在主循环里每圈调用；解析不放在中断里，避免拖慢控制环。
 */
void app_comm_poll(void);

/**
 * app_comm_update：执行链路看门狗判断，并按周期上报遥测。
 * dt_ms：距上次调用的毫秒数，0 时不做任何事。返回：无。
 * 应与 app_line_follow_update / chassis_update 同频调用。
 */
void app_comm_update(uint32_t dt_ms);

/**
 * app_comm_get_mode：查询当前运行模式。
 * 参数：无。返回：MODE_* 之一。
 */
uint8_t app_comm_get_mode(void);

/**
 * app_comm_is_online：链路是否正常。
 * 参数：无。返回：最近 APP_COMM_WATCHDOG_MS 内收到过合法帧为 true。
 */
bool app_comm_is_online(void);

/**
 * app_comm_get_stats：读取通信统计，用于调试与联调。
 * rx_frames：合法接收帧数；crc_errors：校验失败帧数；tx_frames：已发送帧数。
 * 返回：无。任一指针可为 NULL。
 */
void app_comm_get_stats(uint32_t *rx_frames, uint32_t *crc_errors, uint32_t *tx_frames);

#endif /* APP_COMM_H */
