/**
 * @file    protocol.h
 * @brief   上下位机通信协议：帧编解码与 CRC16
 *
 * 与上位机 robot/host/protocol.py 严格对应，改一端必须同步改另一端。
 *
 * 帧格式（大端）
 *   0xAA 0x55 | SEQ | CMD | LEN | PAYLOAD(LEN) | CRC_H | CRC_L
 *   CRC16 = CCITT-FALSE（poly 0x1021，init 0xFFFF），覆盖 SEQ..PAYLOAD
 */
#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include "app_lift.h"

#include <stdint.h>

#define PROTO_SOF1 0xAAu
#define PROTO_SOF2 0x55u
#define PROTO_MAX_PAYLOAD 64u

/* 上位机 -> 下位机 */
#define CMD_VEL 0x10u   /* <float32 v_mm_s, float32 w_rad_s>，仅 MANUAL 模式有效 */
#define CMD_MODE 0x11u  /* <uint8 mode> */
#define CMD_ESTOP 0x12u /* 空 */
#define CMD_PING 0x13u  /* 空；上位机 20Hz 常发，用于喂看门狗 */

#define CMD_LIFT_SET_ZERO 0x14u /* 空，当前位置设零 */
#define CMD_LIFT_MOVE 0x15u     /* <uint16 target_mm> */
#define CMD_LIFT_STOP 0x16u     /* 空，停止升降 */
#define CMD_LIFT_CONFIG 0x17u   /* <float32 counts_per_mm, uint16 max_height_mm> */

/* 下位机 -> 上位机 */
#define FB_STATE 0x80u /* 36 字节状态 */
#define FB_ACK 0x81u   /* <uint8 acked_cmd> */
#define FB_LIFT_STATE 0x82u /* 升降状态，独立于 FB_STATE */

/* FB_LIFT_STATE 的位置有效性标志。 */
#define LIFT_FLAG_CONFIGURED (1u << 0)
#define LIFT_FLAG_REFERENCED (1u << 1)

/* 运行模式，与上位机的 MODE_* 一一对应 */
#define MODE_IDLE 0u        /* 电机断使能，可手推 */
#define MODE_LINE_FOLLOW 1u /* MCU 自主循迹，忽略 CMD_VEL */
#define MODE_MANUAL 2u      /* 跟随 CMD_VEL */
#define MODE_ESTOP 3u       /* 刹车锁死，需显式切回 IDLE 才解除 */

/* status 位 */
#define ST_ENCODER (1u << 0)
#define ST_LOW_VOLT (1u << 1)
#define ST_WATCHDOG (1u << 2)
#define ST_ESTOP (1u << 3)
#define ST_STALL (1u << 4)

/** FB_STATE 的 payload 长度，必须和上位机 struct ">IBBBfffffiiB" 一致 */
#define FB_STATE_LEN 36u
#define FB_LIFT_STATE_LEN 20u

/** proto_cb_t：解析出完整帧后的回调集合；任一成员可为 NULL。 */
typedef struct {
    /** on_vel：收到速度指令。v_mm_s 线速度毫米/秒，w_rad_s 角速度弧度/秒。 */
    void (*on_vel)(float v_mm_s, float w_rad_s);
    /** on_mode：收到模式切换。mode 取值见 MODE_*。 */
    void (*on_mode)(uint8_t mode);
    /** on_estop：收到急停。 */
    void (*on_estop)(void);
    /** cmd 为升降指令；counts_per_mm 仅供 CONFIG 使用，mm 为行程上限或目标高度。 */
    void (*on_lift_command)(uint8_t cmd, float counts_per_mm, uint16_t mm);
} proto_cb_t;

/** proto_state_t：逐字节解析器的当前状态。 */
typedef enum {
    PS_SOF1 = 0,
    PS_SOF2,
    PS_SEQ,
    PS_CMD,
    PS_LEN,
    PS_PAYLOAD,
    PS_CRC_H,
    PS_CRC_L,
} proto_state_t;

/** proto_t：解析器上下文；一个串口一份，不要跨上下文并发使用。 */
typedef struct {
    proto_state_t state; /* 解析器状态 */
    uint8_t seq;         /* 当前帧序号 */
    uint8_t cmd;         /* 当前帧命令字 */
    uint8_t len;         /* 当前帧 payload 长度 */
    uint8_t idx;         /* payload 写入位置 */
    uint16_t crc_rx;     /* 收到帧携带的 CRC */
    uint8_t payload[PROTO_MAX_PAYLOAD]; /* payload 缓存 */
    proto_cb_t cb;       /* 回调集合 */
    uint32_t rx_frames;  /* 累计合法帧数 */
    uint32_t crc_errors; /* 累计 CRC 错误帧数 */
} proto_t;

/**
 * proto_init：清空解析器并绑定回调。
 * p：解析器上下文；cb：回调集合，可为 NULL（则只统计不回调）。
 * 返回：无。
 */
void proto_init(proto_t *p, const proto_cb_t *cb);

/**
 * proto_feed：送入一个收到的字节，凑齐一帧时触发回调。
 * p：解析器上下文；byte：本次收到的字节。
 * 返回：无。函数很短，可在中断里调用，但推荐在主循环批量送入。
 */
void proto_feed(proto_t *p, uint8_t byte);

/**
 * proto_build_state：组一帧 FB_STATE 写入 out。
 * out：至少 43 字节的输出缓冲；seq：帧序号；tick_ms：下位机毫秒计数；
 * mode：当前模式；lf_state：循迹状态；line_mask：五路位图（bit0 最左，1 为黑线）；
 * lf_error：位置偏差；v_cmd/w_cmd：当前速度目标；v_l/v_r：左右轮实际速度；
 * enc_l/enc_r：左右轮累计计数；status：状态位。
 * 返回：写入的字节数（固定 43）。
 */
uint16_t proto_build_state(uint8_t *out, uint8_t seq, uint32_t tick_ms, uint8_t mode,
                           uint8_t lf_state, uint8_t line_mask, float lf_error, float v_cmd,
                           float w_cmd, float v_l, float v_r, int32_t enc_l, int32_t enc_r,
                           uint8_t status);

/**
 * proto_build_ack：组一帧 FB_ACK。
 * out：至少 8 字节的缓冲；seq：帧序号；acked_cmd：被确认的命令字。
 * 返回：写入的字节数（固定 8）。
 */
uint16_t proto_build_ack(uint8_t *out, uint8_t seq, uint8_t acked_cmd);

/** 把升降状态写入 out（至少 27 字节），返回帧长。
 * seq 为反馈帧序号，tick_ms 为毫秒计数，lift 为状态快照。
 * last_cmd、last_cmd_seq 和 result 分别是最近一条指令的命令字、序号和执行结果。 */
uint16_t proto_build_lift_state(uint8_t *out, uint8_t seq, uint32_t tick_ms,
                               const lift_status_t *lift, uint8_t last_cmd, uint8_t last_cmd_seq,
                               lift_command_result_t result);

/**
 * proto_crc16：计算 CRC-16/CCITT-FALSE。
 * data：数据首地址；len：字节数。返回：16 位校验值。
 */
uint16_t proto_crc16(const uint8_t *data, uint16_t len);

#endif /* APP_PROTOCOL_H */
