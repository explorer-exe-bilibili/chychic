/**
 * @file    protocol.c
 * @brief   协议解析与组帧实现
 */
#include "protocol.h"

#include <string.h>

uint16_t proto_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (uint8_t bit = 0; bit < 8u; bit++) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ---- 大端写字节 -------------------------------------------------------- */

static void put_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)(value);
}

/** 将 value 写入 p 指向的两个字节，高字节在前。 */
static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

/** 读取 p 指向的大端无符号 16 位值。 */
static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] << 8) | p[1];
}

static void put_i32(uint8_t *p, int32_t value)
{
    put_u32(p, (uint32_t)value);
}

static void put_f32(uint8_t *p, float value)
{
    union {
        float f;
        uint32_t u;
    } conv;
    conv.f = value;
    put_u32(p, conv.u);
}

static float get_f32(const uint8_t *p)
{
    union {
        float f;
        uint32_t u;
    } conv;
    conv.u = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) |
             (uint32_t)p[3];
    return conv.f;
}

/* ---- 解析 -------------------------------------------------------------- */

/**
 * reset_parser：把解析器退回等帧头状态。
 * p：解析器上下文。返回：无。
 */
static void reset_parser(proto_t *p)
{
    p->state = PS_SOF1;
    p->len = 0;
    p->idx = 0;
    p->crc_rx = 0;
}

/**
 * dispatch_frame：按命令字分发已校验的帧。
 * p：解析器上下文。返回：无。
 */
static void dispatch_frame(proto_t *p)
{
    switch (p->cmd) {
    case CMD_VEL:
        if (p->len != 8u) {
            return;
        }
        if (p->cb.on_vel != NULL) {
            p->cb.on_vel(get_f32(&p->payload[0]), get_f32(&p->payload[4]));
        }
        break;
    case CMD_MODE:
        if (p->len != 1u) {
            return;
        }
        if (p->cb.on_mode != NULL) {
            p->cb.on_mode(p->payload[0]);
        }
        break;
    case CMD_ESTOP:
        if (p->len != 0u) {
            return;
        }
        if (p->cb.on_estop != NULL) {
            p->cb.on_estop();
        }
        break;
    case CMD_PING:
        if (p->len != 0u) {
            return;
        }
        break;
    case CMD_LIFT_SET_ZERO:
    case CMD_LIFT_STOP:
        if (p->len != 0u) {
            return;
        }
        if (p->cb.on_lift_command != NULL) {
            p->cb.on_lift_command(p->cmd, 0.0f, 0u);
        }
        break;
    case CMD_LIFT_MOVE:
        if (p->len != 2u) {
            return;
        }
        if (p->cb.on_lift_command != NULL) {
            p->cb.on_lift_command(p->cmd, 0.0f, get_u16(p->payload));
        }
        break;
    case CMD_LIFT_CONFIG:
        if (p->len != 6u) {
            return;
        }
        if (p->cb.on_lift_command != NULL) {
            p->cb.on_lift_command(p->cmd, get_f32(p->payload), get_u16(&p->payload[4]));
        }
        break;
    default:
        return;
    }
    p->rx_frames++;
}

void proto_init(proto_t *p, const proto_cb_t *cb)
{
    memset(p, 0, sizeof(*p));
    p->state = PS_SOF1;
    if (cb != NULL) {
        p->cb = *cb;
    }
}

void proto_feed(proto_t *p, uint8_t byte)
{
    switch (p->state) {
    case PS_SOF1:
        if (byte == PROTO_SOF1) {
            p->state = PS_SOF2;
        }
        break;

    case PS_SOF2:
        /* AA AA 55 时不能把第二个 AA 吞掉，否则会丢一次同步机会 */
        if (byte == PROTO_SOF2) {
            p->state = PS_SEQ;
        } else if (byte != PROTO_SOF1) {
            p->state = PS_SOF1;
        }
        break;

    case PS_SEQ:
        p->seq = byte;
        p->state = PS_CMD;
        break;

    case PS_CMD:
        p->cmd = byte;
        p->state = PS_LEN;
        break;

    case PS_LEN:
        if (byte > PROTO_MAX_PAYLOAD) {
            reset_parser(p); /* 长度非法，当作假帧头 */
            break;
        }
        p->len = byte;
        p->idx = 0;
        p->state = (byte == 0u) ? PS_CRC_H : PS_PAYLOAD;
        break;

    case PS_PAYLOAD:
        p->payload[p->idx++] = byte;
        if (p->idx >= p->len) {
            p->state = PS_CRC_H;
        }
        break;

    case PS_CRC_H:
        p->crc_rx = (uint16_t)((uint16_t)byte << 8);
        p->state = PS_CRC_L;
        break;

    case PS_CRC_L: {
        p->crc_rx |= byte;

        /* CRC 覆盖 SEQ | CMD | LEN | PAYLOAD，按同样顺序重算 */
        uint8_t buf[3u + PROTO_MAX_PAYLOAD];
        buf[0] = p->seq;
        buf[1] = p->cmd;
        buf[2] = p->len;
        if (p->len > 0u) {
            memcpy(&buf[3], p->payload, p->len);
        }

        if (proto_crc16(buf, (uint16_t)(3u + p->len)) == p->crc_rx) {
            dispatch_frame(p);
        } else {
            p->crc_errors++;
        }
        reset_parser(p);
        break;
    }

    default:
        reset_parser(p);
        break;
    }
}

/* ---- 组帧 -------------------------------------------------------------- */

/**
 * finish_frame：给已写好的 out[0..body_end-1] 补 CRC。
 * out：输出缓冲；body_end：payload 结束后的下一个下标。
 * 返回：整帧长度（body_end + 2）。
 */
static uint16_t finish_frame(uint8_t *out, uint16_t body_end)
{
    uint16_t crc = proto_crc16(&out[2], (uint16_t)(body_end - 2u));
    out[body_end] = (uint8_t)(crc >> 8);
    out[body_end + 1] = (uint8_t)(crc & 0xFFu);
    return (uint16_t)(body_end + 2u);
}

uint16_t proto_build_state(uint8_t *out, uint8_t seq, uint32_t tick_ms, uint8_t mode,
                           uint8_t lf_state, uint8_t line_mask, float lf_error, float v_cmd,
                           float w_cmd, float v_l, float v_r, int32_t enc_l, int32_t enc_r,
                           uint8_t status)
{
    out[0] = PROTO_SOF1;
    out[1] = PROTO_SOF2;
    out[2] = seq;
    out[3] = FB_STATE;
    out[4] = (uint8_t)FB_STATE_LEN;

    put_u32(&out[5], tick_ms);   /* 0  */
    out[9] = mode;               /* 4  */
    out[10] = lf_state;          /* 5  */
    out[11] = line_mask;         /* 6  */
    put_f32(&out[12], lf_error); /* 7  */
    put_f32(&out[16], v_cmd);    /* 11 */
    put_f32(&out[20], w_cmd);    /* 15 */
    put_f32(&out[24], v_l);      /* 19 */
    put_f32(&out[28], v_r);      /* 23 */
    put_i32(&out[32], enc_l);    /* 27 */
    put_i32(&out[36], enc_r);    /* 31 */
    out[40] = status;            /* 35 */

    return finish_frame(out, 41u);
}

uint16_t proto_build_lift_state(uint8_t *out, uint8_t seq, uint32_t tick_ms,
                               const lift_status_t *lift, uint8_t last_cmd, uint8_t last_cmd_seq,
                               lift_command_result_t result)
{
    out[0] = PROTO_SOF1;
    out[1] = PROTO_SOF2;
    out[2] = seq;
    out[3] = FB_LIFT_STATE;
    out[4] = FB_LIFT_STATE_LEN;
    put_u32(&out[5], tick_ms);
    put_f32(&out[9], lift->position_mm);
    put_u16(&out[13], lift->target_mm);
    put_i32(&out[15], lift->encoder_count);
    out[19] = (uint8_t)lift->state;
    out[20] = (uint8_t)lift->fault;
    out[21] = (lift->configured ? LIFT_FLAG_CONFIGURED : 0u) |
              (lift->referenced ? LIFT_FLAG_REFERENCED : 0u);
    out[22] = last_cmd;
    out[23] = last_cmd_seq;
    out[24] = (uint8_t)result;
    return finish_frame(out, 5u + FB_LIFT_STATE_LEN);
}

uint16_t proto_build_ack(uint8_t *out, uint8_t seq, uint8_t acked_cmd)
{
    out[0] = PROTO_SOF1;
    out[1] = PROTO_SOF2;
    out[2] = seq;
    out[3] = FB_ACK;
    out[4] = 1u;
    out[5] = acked_cmd;
    return finish_frame(out, 6u);
}
