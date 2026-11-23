/**
 * @file    protocol.c
 */
#include "protocol.h"
#include <string.h>

uint16_t proto_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (uint8_t b = 0; b < 8u; b++)
        {
            if (crc & 0x8000u)
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ---- 大端写入 ---------------------------------------------------------- */

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v);
}

static void put_i32(uint8_t *p, int32_t v)
{
    put_u32(p, (uint32_t)v);
}

/* 浮点按位搬运：C 里用 union 避免严格别名问题 */
static void put_f32(uint8_t *p, float f)
{
    union { float f; uint32_t u; } cv;
    cv.f = f;
    put_u32(p, cv.u);
}

static float get_f32(const uint8_t *p)
{
    union { float f; uint32_t u; } cv;
    cv.u = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
    return cv.f;
}

/* ---- 初始化 ------------------------------------------------------------ */

void proto_init(proto_t *p, const proto_cb_t *cb)
{
    memset(p, 0, sizeof(*p));
    p->state = PS_SOF1;
    if (cb != NULL)
        p->cb = *cb;
}

/* ---- 解析 -------------------------------------------------------------- */

static void reset_parser(proto_t *p)
{
    p->state = PS_SOF1;
    p->len = 0;
    p->idx = 0;
    p->crc_calc = 0xFFFFu;
    p->crc_rx = 0;
}

static void dispatch(proto_t *p)
{
    switch (p->cmd)
    {
    case CMD_VEL:
        if (p->len == 8u && p->cb.on_vel != NULL)
            p->cb.on_vel(get_f32(&p->payload[0]), get_f32(&p->payload[4]));
        break;
    case CMD_MODE:
        if (p->len == 1u && p->cb.on_mode != NULL)
            p->cb.on_mode(p->payload[0]);
        break;
    case CMD_ESTOP:
        if (p->cb.on_estop != NULL)
            p->cb.on_estop();
        break;
    default:
        break;
    }
    p->rx_frames++;
}

void proto_feed(proto_t *p, uint8_t byte)
{
    switch (p->state)
    {
    case PS_SOF1:
        if (byte == PROTO_SOF1)
            p->state = PS_SOF2;
        break;

    case PS_SOF2:
        if (byte == PROTO_SOF2)
            p->state = PS_SEQ;
        else if (byte != PROTO_SOF1)   /* AA AA 55 时不能把第二个 AA 吞掉 */
            p->state = PS_SOF1;
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
        if (byte > PROTO_MAX_PAYLOAD)
        {
            reset_parser(p);       /* 假帧头，重来 */
            break;
        }
        p->len = byte;
        p->idx = 0;
        p->state = (byte == 0u) ? PS_CRC_H : PS_PAYLOAD;
        break;

    case PS_PAYLOAD:
        p->payload[p->idx++] = byte;
        if (p->idx >= p->len)
            p->state = PS_CRC_H;
        break;

    case PS_CRC_H:
        p->crc_rx = (uint16_t)((uint16_t)byte << 8);
        p->state = PS_CRC_L;
        break;

    case PS_CRC_L:
    {
        p->crc_rx |= byte;

        /* CRC 覆盖 SEQ|CMD|LEN|PAYLOAD，这里按同样顺序重算 */
        uint8_t buf[3u + PROTO_MAX_PAYLOAD];
        buf[0] = p->seq;
        buf[1] = p->cmd;
        buf[2] = p->len;
        if (p->len > 0u)
            memcpy(&buf[3], p->payload, p->len);

        p->crc_calc = proto_crc16(buf, (uint16_t)(3u + p->len));
        if (p->crc_calc == p->crc_rx)
        {
            dispatch(p);
        }
        else
        {
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

static uint16_t finish_frame(uint8_t *out, uint16_t body_len)
{
    uint16_t crc = proto_crc16(&out[2], (uint16_t)(body_len - 2u));
    out[body_len]     = (uint8_t)(crc >> 8);
    out[body_len + 1] = (uint8_t)(crc & 0xFFu);
    return (uint16_t)(body_len + 2u);
}

uint16_t proto_build_state(uint8_t *out, uint8_t seq,
                           uint32_t tick_ms,
                           float v_l, float v_r,
                           int32_t enc_l, int32_t enc_r,
                           uint8_t ir, uint8_t status)
{
    out[0] = PROTO_SOF1;
    out[1] = PROTO_SOF2;
    out[2] = seq;
    out[3] = FB_STATE;
    out[4] = FB_STATE_LEN;

    put_u32(&out[5], tick_ms);
    put_f32(&out[9], v_l);
    put_f32(&out[13], v_r);
    put_i32(&out[17], enc_l);
    put_i32(&out[21], enc_r);
    out[25] = ir;
    out[26] = status;

    /* body = SEQ..PAYLOAD = out[2..26]，共 25 字节 */
    return finish_frame(out, 27u);
}
