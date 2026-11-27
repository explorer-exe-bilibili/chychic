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
#include "app_comm_text.h"

#include "app_lift.h"
#include "app_line_follow.h"
#include "app_motor.h"
#include "chassis.h"
#include "chassis_config.h"
#include "gettick.h"
#include "motor_speed.h"
#include "motor_types.h"
#include "protocol.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/** 接收环形缓冲大小，字节。20Hz × 11 字节远小于此值，留足余量。 */
#define APP_COMM_RX_BUF 128u
/* 容纳二进制反馈帧，或四条曲线的一组数据和一行文本应答。 */
#define APP_COMM_TX_BUF 384u

static UART_HandleTypeDef *s_huart;              /* 绑定的串口句柄 */
static proto_t s_proto;                          /* 协议解析器 */
static uint8_t s_rx_buf[APP_COMM_RX_BUF];        /* DMA 循环接收缓冲 */
static volatile uint16_t s_rx_tail;              /* 主循环已解析到的位置 */
static volatile uint16_t s_it_rx_head;           /* 中断回退模式的接收写入位置 */
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
static uint32_t s_rx_errors;                     /* 串口错误回调触发次数 */

static uint8_t s_last_lift_cmd;                  /* 最近处理的升降命令字 */
static uint8_t s_last_lift_cmd_seq;              /* 最近处理的升降命令序号 */
static lift_command_result_t s_lift_cmd_result;  /* 最近升降命令的执行结果 */

typedef enum {
    TEXT_REPLY_NONE = 0,
    TEXT_REPLY_PLOT_ON,
    TEXT_REPLY_PID,
    TEXT_REPLY_INVALID,
    TEXT_REPLY_ESTOP,
} text_reply_t;

static comm_text_rx_t s_text_rx;  /* 文本命令的分片接收状态。 */
static bool s_plot_mode;         /* true 表示 UART1 使用文本遥测。 */
static text_reply_t s_text_reply; /* 待发送的最近一条应答。 */

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
        app_lift_stop(LIFT_FAULT_ESTOP);
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
    if (s_mode == MODE_ESTOP && mode != MODE_IDLE) {
        return;
    }
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

/** 处理已校验的升降命令。cmd 为命令字；counts_per_mm 和 mm 为标定或定位参数。 */
static void on_lift_command(uint8_t cmd, float counts_per_mm, uint16_t mm)
{
    s_last_lift_cmd = cmd;
    s_last_lift_cmd_seq = s_proto.seq;

    if (cmd == CMD_LIFT_MOVE && (s_status & ST_ESTOP) != 0u) {
        s_lift_cmd_result = LIFT_COMMAND_ESTOP;
        return;
    }

    switch (cmd) {
    case CMD_LIFT_CONFIG:
        s_lift_cmd_result = app_lift_configure(counts_per_mm, mm);
        break;
    case CMD_LIFT_SET_ZERO:
        s_lift_cmd_result = app_lift_set_zero();
        break;
    case CMD_LIFT_MOVE:
        s_lift_cmd_result = app_lift_move(mm, Get_tick());
        break;
    case CMD_LIFT_STOP:
        app_lift_stop(LIFT_FAULT_NONE);
        s_lift_cmd_result = LIFT_COMMAND_OK;
        break;
    default:
        break;
    }
}

/** 执行以文本结束符收齐的 line，返回命令是否有效。 */
static bool on_text_command(char *line)
{
    comm_text_command_t command = app_comm_text_parse(line);
    if (command.action == COMM_TEXT_PLOT_ON) {
        s_plot_mode = true;
        s_text_reply = TEXT_REPLY_PLOT_ON;
        return true;
    }
    if (!s_plot_mode) {
        return false;
    }

    switch (command.action) {
    case COMM_TEXT_PLOT_OFF:
        s_plot_mode = false;
        s_text_reply = TEXT_REPLY_NONE;
        return true;

    case COMM_TEXT_PID_GET:
        s_text_reply = TEXT_REPLY_PID;
        return true;

    case COMM_TEXT_PID_SET: {
        bool applied;
        if (command.wheel == MOTOR_COUNT) {
            applied = app_motor_set_speed_pid(MOTOR_LEFT, &command.gains) &&
                      app_motor_set_speed_pid(MOTOR_RIGHT, &command.gains);
        } else {
            applied = app_motor_set_speed_pid(command.wheel, &command.gains);
        }
        s_text_reply = applied ? TEXT_REPLY_PID : TEXT_REPLY_INVALID;
        return applied;
    }

    case COMM_TEXT_SPEED:
        if (s_mode == MODE_ESTOP) {
            s_text_reply = TEXT_REPLY_ESTOP;
            return false;
        }
        if (fabsf(command.speed_mm_s) > CHASSIS_MAX_WHEEL_SPEED_MM_S) {
            s_text_reply = TEXT_REPLY_INVALID;
            return false;
        }
        on_mode(MODE_MANUAL);
        on_vel(command.speed_mm_s, 0.0f);
        return true;

    case COMM_TEXT_IDLE:
        s_v_cmd = 0.0f;
        s_w_cmd = 0.0f;
        on_mode(MODE_IDLE);
        return true;

    case COMM_TEXT_ESTOP:
        on_estop();
        return true;

    case COMM_TEXT_PING:
        return true;

    default:
        s_text_reply = TEXT_REPLY_INVALID;
        return false;
    }
}

/** now_ms 为当前毫秒计数。链路超时后停止底盘和丝杆。 */
static void update_watchdog(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - s_last_rx_ms) <= APP_COMM_WATCHDOG_MS) {
        return;
    }

    app_lift_stop(LIFT_FAULT_LINK_LOST);
    if (s_mode != MODE_IDLE && s_mode != MODE_ESTOP) {
        apply_mode(MODE_IDLE);
    }
    s_status |= ST_WATCHDOG;
}

/* ---- 内部：遥测 -------------------------------------------------------- */

/**
 * 在发送缓冲中组帧。now_ms 为当前毫秒计数，feedback_left/right 为最近轮速反馈。
 * 返回二进制反馈总长度，包含底盘状态和丝杆状态。
 */
static uint16_t build_binary_telemetry(uint32_t now_ms, const motor_feedback_t *feedback_left,
                                       const motor_feedback_t *feedback_right)
{
    line_follow_status_t line_status = {0};
    (void)app_line_follow_get_status(&line_status);

    /* 循迹模式下回报循迹模块的目标，手动模式下回报上位机下发的目标。 */
    float v_cmd_mm_s = (s_mode == MODE_LINE_FOLLOW) ? line_status.linear_speed_mm_s : s_v_cmd;
    float w_cmd_rad_s =
        (s_mode == MODE_LINE_FOLLOW) ? line_status.angular_speed_rad_s : s_w_cmd;

    uint16_t length =
        proto_build_state(s_tx_buf, s_tx_seq++, now_ms, s_mode, (uint8_t)line_status.state,
                          line_status.line_mask, line_status.position_error, v_cmd_mm_s,
                          w_cmd_rad_s, feedback_left->filtered_speed_mm_s,
                          feedback_right->filtered_speed_mm_s, (int32_t)feedback_left->total_counts,
                          (int32_t)feedback_right->total_counts, s_status);

    lift_status_t lift_status = app_lift_get_status();
    return length + proto_build_lift_state(&s_tx_buf[length], s_tx_seq++, now_ms, &lift_status,
                                            s_last_lift_cmd, s_last_lift_cmd_seq, s_lift_cmd_result);
}

/** feedback_left/right 为最近轮速反馈。返回一组曲线及待发应答的总字节数。 */
static uint16_t build_plot_telemetry(const motor_feedback_t *feedback_left,
                                     const motor_feedback_t *feedback_right)
{
    float left_target;
    float right_target;
    chassis_get_wheel_targets(&left_target, &right_target);
    char *out = (char *)s_tx_buf;
    size_t length = app_comm_text_plot(out, sizeof(s_tx_buf), left_target,
                                        feedback_left->filtered_speed_mm_s, right_target,
                                        feedback_right->filtered_speed_mm_s);
    if (length == 0U || s_text_reply == TEXT_REPLY_NONE) {
        return (uint16_t)length;
    }

    size_t remaining = sizeof(s_tx_buf) - length;
    int reply_length;
    if (s_text_reply == TEXT_REPLY_PID) {
        pid_gains_t left = {0};
        pid_gains_t right = {0};
        (void)app_motor_get_speed_pid(MOTOR_LEFT, &left);
        (void)app_motor_get_speed_pid(MOTOR_RIGHT, &right);
        /* 括号内的参数不会被网页识别为绘图用的数值。 */
        reply_length = snprintf(out + length, remaining,
                                "OK PID left kp(%.9g) ki(%.9g) kd(%.9g) "
                                "right kp(%.9g) ki(%.9g) kd(%.9g)\n",
                                (double)left.kp, (double)left.ki, (double)left.kd,
                                (double)right.kp, (double)right.ki, (double)right.kd);
    } else {
        static const char *const replies[] = {
            [TEXT_REPLY_PLOT_ON] = "OK PLOT ON\n",
            [TEXT_REPLY_INVALID] = "ERR INVALID COMMAND OR VALUE\n",
            [TEXT_REPLY_ESTOP] = "ERR ESTOP SEND !idle#\n",
        };
        reply_length = snprintf(out + length, remaining, "%s", replies[s_text_reply]);
    }
    if (reply_length < 0 || (size_t)reply_length >= remaining) {
        return 0;
    }
    return (uint16_t)(length + (size_t)reply_length);
}

/** now_ms 为当前毫秒计数。通过 DMA 发送到期的遥测，串口忙时跳过。 */
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

    motor_feedback_t feedback_left = {0};
    motor_feedback_t feedback_right = {0};
    (void)motor_speed_get_feedback(MOTOR_LEFT, &feedback_left);
    (void)motor_speed_get_feedback(MOTOR_RIGHT, &feedback_right);
    uint16_t length = s_plot_mode ? build_plot_telemetry(&feedback_left, &feedback_right)
                                  : build_binary_telemetry(now_ms, &feedback_left, &feedback_right);
    if (length > 0U && HAL_UART_Transmit_DMA(s_huart, s_tx_buf, length) == HAL_OK) {
        s_tx_frames++;
        s_text_reply = TEXT_REPLY_NONE;
    } else {
        s_tx_dropped++;
    }
}

/* ---- 对外接口 ---------------------------------------------------------- */

void app_comm_init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_text_rx = (comm_text_rx_t){0};
    s_plot_mode = false;
    s_text_reply = TEXT_REPLY_NONE;
    s_rx_tail = 0;
    s_it_rx_head = 0;
    s_tx_seq = 0;
    s_last_lift_cmd = 0;
    s_last_lift_cmd_seq = 0;
    s_lift_cmd_result = LIFT_COMMAND_OK;
    s_mode = MODE_IDLE;
    s_status = ST_ENCODER;
    s_v_cmd = 0.0f;
    s_w_cmd = 0.0f;
    s_tx_frames = 0;
    s_tx_dropped = 0;

    proto_cb_t callbacks = {
        .on_vel = on_vel,
        .on_mode = on_mode,
        .on_estop = on_estop,
        .on_lift_command = on_lift_command,
    };
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

    /* 先处理已经发生的断联，再消费新到的帧。 */
    update_watchdog(Get_tick());
    uint32_t frames_before = s_proto.rx_frames;
    bool valid_text_command = false;

    uint16_t head = s_it_rx_head;
    if (s_dma_rx_ok && s_huart->hdmarx != NULL) {
        /* CNDTR 递减，剩余越少说明写入越多 */
        uint16_t remain = (uint16_t)__HAL_DMA_GET_COUNTER(s_huart->hdmarx);
        head = (uint16_t)((APP_COMM_RX_BUF - remain) % APP_COMM_RX_BUF);
    }

    while (s_rx_tail != head) {
        uint8_t byte = s_rx_buf[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1u) % APP_COMM_RX_BUF);

        char *line;
        if (app_comm_text_feed(&s_text_rx, byte, s_proto.state == PS_SOF1, &line)) {
            if (line != NULL && on_text_command(line)) {
                valid_text_command = true;
            }
        } else {
            proto_feed(&s_proto, byte);
        }
    }

    /* 有效的二进制帧和文本命令都可维持链路。 */
    if (valid_text_command || s_proto.rx_frames != frames_before) {
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

    update_watchdog(now_ms);

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

    /* 回调只排队；DMA 和中断回退都在主循环解析，避免在 ISR 里改升降状态。 */
    uint16_t next_head = (uint16_t)((s_it_rx_head + 1u) % APP_COMM_RX_BUF);
    if (next_head != s_rx_tail) {
        s_rx_buf[s_it_rx_head] = s_it_byte;
        s_it_rx_head = next_head;
    }
    (void)HAL_UART_Receive_IT(huart, &s_it_byte, 1);
}

/**
 * HAL_UART_ErrorCallback：串口错误处理。
 * huart：触发回调的串口句柄。返回：无。
 *
 * 这是开启 USART1 全局中断后必须实现的部分。HAL 的 UART_IRQHandler 在检测到
 * 溢出（ORE）/ 帧错 / 噪声时会先调用 UART_EndRxTransfer() 把接收停掉，再回调本函数。
 * 如果这里不重新武装，一次总线毛刺就会让接收永久失效——现象是"跑一阵突然
 * 再也收不到上位机指令"，而且不报任何错。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) {
        return;
    }

    s_rx_errors++;
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    if (s_dma_rx_ok) {
        /* DMA 环形缓冲从头重启；已解析位置一并归零。 */
        s_rx_tail = 0;
        __HAL_UART_CLEAR_OREFLAG(huart);
        (void)HAL_UART_Receive_DMA(huart, s_rx_buf, APP_COMM_RX_BUF);
    } else {
        (void)HAL_UART_Receive_IT(huart, &s_it_byte, 1);
    }
}
