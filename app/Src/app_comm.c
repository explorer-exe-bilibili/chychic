/**
 * @file    app_comm.c
 * @brief   上下位机通信实现
 *
 * 接收用 DMA 循环模式：一次配置永久有效，主循环轮询 DMA 剩余计数取新字节，
 * 不产生逐字节中断，也不会因为其它中断屏蔽而丢数据。
 *
 * 发送用 DMA 普通模式：发完才置回 READY，因此发送前必须检查 gState，
 * 忙就丢弃这一帧——遥测少一帧无所谓，控制环不能被阻塞。
 */
#include "app_comm.h"

#include "app_line_follow.h"
#include "chassis.h"
#include "gettick.h"
#include "motor_speed.h"
#include "motor_types.h"
#include "protocol.h"

#include <string.h>

/** 接收环形缓冲大小，字节。20Hz × 11 字节远小于此值，留足余量。 */
#define APP_COMM_RX_BUF 128u
/** 单帧最大长度：2 帧头 + 3 头 + 36 载荷 + 2 CRC */
#define APP_COMM_TX_BUF 48u

static UART_HandleTypeDef *s_huart;              /* 绑定的串口句柄 */
static proto_t s_proto;                          /* 协议解析器 */
static uint8_t s_rx_buf[APP_COMM_RX_BUF];        /* DMA 循环接收缓冲 */
static uint16_t s_rx_tail;                       /* 主循环已解析到的位置 */
static uint8_t s_tx_buf[APP_COMM_TX_BUF];        /* 遥测组帧缓冲 */
static uint8_t s_tx_seq;                         /* 发送帧序号 */
static bool s_dma_rx_ok;                         /* 循环 DMA 是否启动成功 */
static uint8_t s_it_byte;                        /* DMA 失败时的单字节回退缓冲 */

static uint8_t s_mode;                           /* 当前运行模式 */
static uint8_t s_status;                         /* 上报的状态位 */
static float s_v_cmd;                            /* MANUAL 模式下的线速度目标 mm/s */
static float s_w_cmd;                            /* MANUAL 模式下的角速度目标 rad/s */
static uint32_t s_last_rx_ms;                    /* 最近一次收到合法帧的时刻 */
static uint32_t s_last_tx_ms;                    /* 最近一次发送遥测的时刻 */
static uint32_t s_tx_frames;                     /* 已发送帧数 */
static uint32_t s_tx_dropped;                    /* 因串口忙而丢弃的帧数 */
static uint32_t s_rx_overrun;                    /* 接收溢出次数，正常应为 0 */

/* ---- 内部：模式切换 ---------------------------------------------------- */

/**
 * apply_mode：执行模式切换对应的动作。
 * mode：目标模式，取值见 MODE_*。返回：无。
 * 重复设置同一模式直接返回，避免反复启停循迹模块。
 */
static void apply_mode(uint8_t mode)
{
    if (mode == s_mode) {
        return;
    }

    switch (mode) {
    case MODE_LINE_FOLLOW:
        /* 交给 MCU 的循迹模块，速度目标由它自己产生 */
        app_line_follow_start();
        break;

    case MODE_MANUAL:
        /* 先停掉循迹，否则它会和上位机指令抢控制权 */
        app_line_follow_stop();
        break;

    case MODE_ESTOP:
        app_line_follow_stop();
        chassis_stop();
        s_status |= ST_ESTOP;
        break;

    case MODE_IDLE:
    default:
        app_line_follow_stop();
        chassis_stop();
        s_status &= (uint8_t)~ST_ESTOP;
        break;
    }

    s_mode = mode;
}

/* ---- 内部：协议回调 ---------------------------------------------------- */

/**
 * on_vel：处理速度指令。
 * v_mm_s：线速度毫米/秒；w_rad_s：角速度弧度/秒。返回：无。
 * 仅在 MANUAL 模式下生效；IDLE/ESTOP 时忽略，防止误动。
 */
static void on_vel(float v_mm_s, float w_rad_s)
{
    s_v_cmd = v_mm_s;
    s_w_cmd = w_rad_s;
    if (s_mode == MODE_MANUAL) {
        chassis_set_velocity(v_mm_s, w_rad_s);
    }
}

/**
 * on_mode：处理模式切换指令。
 * mode：目标模式。返回：无。
 */
static void on_mode(uint8_t mode)
{
    apply_mode(mode);
}

/**
 * on_estop：处理急停指令。
 * 参数：无。返回：无。
 */
static void on_estop(void)
{
    s_v_cmd = 0.0f;
    s_w_cmd = 0.0f;
    apply_mode(MODE_ESTOP);
}

/* ---- 内部：遥测 -------------------------------------------------------- */

/**
 * send_telemetry：组一帧 FB_STATE 并通过 DMA 发出。
 * now_ms：当前毫秒计数。返回：无。
 * 串口忙则丢帧，不等待。
 */
static void send_telemetry(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - s_last_tx_ms) < APP_COMM_TELEMETRY_MS) {
        return;
    }
    s_last_tx_ms = now_ms;

    if (s_huart == NULL || s_huart->gState != HAL_UART_STATE_READY) {
        s_tx_dropped++;
        return;
    }

    line_follow_status_t line_status;
    memset(&line_status, 0, sizeof(line_status));
    (void)app_line_follow_get_status(&line_status);

    motor_feedback_t feedback_left;
    motor_feedback_t feedback_right;
    memset(&feedback_left, 0, sizeof(feedback_left));
    memset(&feedback_right, 0, sizeof(feedback_right));
    (void)motor_speed_get_feedback(MOTOR_LEFT, &feedback_left);
    (void)motor_speed_get_feedback(MOTOR_RIGHT, &feedback_right);

    /* 循迹模式下回报循迹模块的目标，手动模式下回报上位机下发的目标 */
    float v_cmd_mm_s = (s_mode == MODE_LINE_FOLLOW) ? line_status.linear_speed_mm_s : s_v_cmd;
    float w_cmd_rad_s =
        (s_mode == MODE_LINE_FOLLOW) ? line_status.angular_speed_rad_s : s_w_cmd;

    uint16_t length =
        proto_build_state(s_tx_buf, s_tx_seq++, now_ms, s_mode, (uint8_t)line_status.state,
                          line_status.line_mask, line_status.position_error, v_cmd_mm_s,
                          w_cmd_rad_s, feedback_left.filtered_speed_mm_s,
                          feedback_right.filtered_speed_mm_s, (int32_t)feedback_left.total_counts,
                          (int32_t)feedback_right.total_counts, s_status);

    if (HAL_UART_Transmit_DMA(s_huart, s_tx_buf, length) == HAL_OK) {
        s_tx_frames++;
    } else {
        s_tx_dropped++;
    }
}

/* ---- 对外接口 ---------------------------------------------------------- */

void app_comm_init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_rx_tail = 0;
    s_tx_seq = 0;
    s_mode = MODE_IDLE;
    s_status = ST_ENCODER;
    s_v_cmd = 0.0f;
    s_w_cmd = 0.0f;
    s_tx_frames = 0;
    s_tx_dropped = 0;

    proto_cb_t callbacks = {on_vel, on_mode, on_estop};
    proto_init(&s_proto, &callbacks);

    uint32_t now_ms = Get_tick();
    s_last_rx_ms = now_ms;
    s_last_tx_ms = now_ms;

    if (huart == NULL) {
        s_dma_rx_ok = false;
        return;
    }

    /* 循环 DMA：一次武装，之后永远有效，不产生逐字节中断 */
    s_dma_rx_ok = (HAL_UART_Receive_DMA(huart, s_rx_buf, APP_COMM_RX_BUF) == HAL_OK);
    if (!s_dma_rx_ok) {
        /* 回退到单字节中断，至少链路不至于完全哑掉 */
        (void)HAL_UART_Receive_IT(huart, &s_it_byte, 1);
    }
}

void app_comm_poll(void)
{
    if (s_huart == NULL) {
        return;
    }

    uint32_t frames_before = s_proto.rx_frames;

    if (s_dma_rx_ok && s_huart->hdmarx != NULL) {
        /* CNDTR 递减，剩余越少说明写入越多 */
        uint16_t remain = (uint16_t)__HAL_DMA_GET_COUNTER(s_huart->hdmarx);
        uint16_t head = (uint16_t)((APP_COMM_RX_BUF - remain) % APP_COMM_RX_BUF);

        while (s_rx_tail != head) {
            proto_feed(&s_proto, s_rx_buf[s_rx_tail]);
            s_rx_tail = (uint16_t)((s_rx_tail + 1u) % APP_COMM_RX_BUF);
        }
    }

    /* 任何合法帧（包括 PING）都算链路存活 */
    if (s_proto.rx_frames != frames_before) {
        s_last_rx_ms = Get_tick();
        s_status &= (uint8_t)~ST_WATCHDOG;
    }

    /* 防御：NVIC 没开 USART1 全局中断时，溢出标志没人清，会一直置位。
       读 SR 再读 DR 即可清除；顺带统计，便于发现波特率不匹配。 */
    if (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(s_huart);
        s_rx_overrun++;
    }
}

void app_comm_update(uint32_t dt_ms)
{
    if (dt_ms == 0u) {
        return;
    }

    uint32_t now_ms = Get_tick();

    /* 链路看门狗：上位机挂了就停车，不留下没人管的自主小车 */
    if ((uint32_t)(now_ms - s_last_rx_ms) > APP_COMM_WATCHDOG_MS) {
        if (s_mode != MODE_IDLE) {
            apply_mode(MODE_IDLE);
        }
        s_status |= ST_WATCHDOG;
    }

    send_telemetry(now_ms);
}

uint8_t app_comm_get_mode(void)
{
    return s_mode;
}

bool app_comm_is_online(void)
{
    return (uint32_t)(Get_tick() - s_last_rx_ms) <= APP_COMM_WATCHDOG_MS;
}

void app_comm_get_stats(uint32_t *rx_frames, uint32_t *crc_errors, uint32_t *tx_frames)
{
    if (rx_frames != NULL) {
        *rx_frames = s_proto.rx_frames;
    }
    if (crc_errors != NULL) {
        *crc_errors = s_proto.crc_errors;
    }
    if (tx_frames != NULL) {
        *tx_frames = s_tx_frames + s_tx_dropped;
    }
}

/* ---- 回退：DMA 启动失败时的单字节中断 ---------------------------------- */

/**
 * HAL_UART_RxCpltCallback：DMA 不可用时的接收回退。
 * huart：触发回调的串口句柄。返回：无。
 * 必须重新武装接收，否则只能收到第一个字节。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_huart || s_dma_rx_ok) {
        return;
    }

    proto_feed(&s_proto, s_it_byte);
    s_last_rx_ms = Get_tick();
    s_status &= (uint8_t)~ST_WATCHDOG;
    (void)HAL_UART_Receive_IT(huart, &s_it_byte, 1);
}
