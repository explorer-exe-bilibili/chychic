/**
 * @file    protocol.h
 * @brief   上下位机协议 —— 与 host/protocol.py 严格对应，改一端必须改另一端
 *
 * 帧格式（大端）
 *   0xAA 0x55 | SEQ | CMD | LEN | PAYLOAD(LEN) | CRC_H | CRC_L
 *   CRC16 = CCITT-FALSE (poly 0x1021, init 0xFFFF)，覆盖 SEQ..PAYLOAD
 */
#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include <stdint.h>

#define PROTO_SOF1        0xAAu
#define PROTO_SOF2        0x55u
#define PROTO_MAX_PAYLOAD 64u

/* 上位机 -> 下位机 */
#define CMD_VEL           0x10u
#define CMD_MODE          0x11u
#define CMD_ESTOP         0x12u

/* 下位机 -> 上位机 */
#define FB_STATE          0x80u
#define FB_ACK            0x81u

/* mode */
#define MODE_IDLE         0u
#define MODE_RUN          1u
#define MODE_ESTOP        2u
#define MODE_CALIB        3u

/* status 位 */
#define ST_ENCODER        (1u << 0)
#define ST_LOW_VOLT       (1u << 1)
#define ST_WATCHDOG       (1u << 2)
#define ST_ESTOP          (1u << 3)
#define ST_STALL          (1u << 4)

/* FB_STATE 的 payload 长度，必须和 Python 侧 struct ">IffiiBB" 一致 */
#define FB_STATE_LEN      22u

typedef struct
{
    void (*on_vel)(float v, float w);
    void (*on_mode)(uint8_t mode);
    void (*on_estop)(void);
} proto_cb_t;

typedef enum
{
    PS_SOF1 = 0,
    PS_SOF2,
    PS_SEQ,
    PS_CMD,
    PS_LEN,
    PS_PAYLOAD,
    PS_CRC_H,
    PS_CRC_L,
} proto_state_t;

typedef struct
{
    proto_state_t state;
    uint8_t  seq;
    uint8_t  cmd;
    uint8_t  len;
    uint8_t  idx;
    uint16_t crc_calc;
    uint16_t crc_rx;
    uint8_t  payload[PROTO_MAX_PAYLOAD];
    proto_cb_t cb;

    uint32_t rx_frames;
    uint32_t crc_errors;
} proto_t;

void proto_init(proto_t *p, const proto_cb_t *cb);

/** 每收到一个字节调用一次（在 UART 中断里调，函数本身很短） */
void proto_feed(proto_t *p, uint8_t byte);

/**
 * 组一帧 FB_STATE 写入 out。
 * @return 帧总长度
 */
uint16_t proto_build_state(uint8_t *out, uint8_t seq,
                           uint32_t tick_ms,
                           float v_l, float v_r,
                           int32_t enc_l, int32_t enc_r,
                           uint8_t ir, uint8_t status);

uint16_t proto_crc16(const uint8_t *data, uint16_t len);

#endif /* APP_PROTOCOL_H */
