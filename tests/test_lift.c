#include "app_lift.h"
#include "bsp_lift.h"
#include "lift_config.h"
#include "protocol.h"

#include <assert.h>
#include <math.h>

static uint32_t encoder_count;
static int16_t output_permille;
static uint8_t received_command;
static float received_scale;
static uint16_t received_height;

bool bsp_lift_init(void) {
    output_permille = 0;
    return true;
}

void bsp_lift_set_pwm(int16_t pwm_permille) {
    output_permille = pwm_permille;
}

uint32_t bsp_lift_encoder_count(void) {
    return encoder_count;
}

/** 记录接收到的升降命令，cmd、scale、height 为解析后的字段。 */
static void capture_command(uint8_t cmd, float scale, uint16_t height) {
    received_command = cmd;
    received_scale = scale;
    received_height = height;
}

/** 验证默认锁定、手动设零、减速、双向定位和停机。 */
static void test_positioning(void) {
    assert(app_lift_init());
    assert(app_lift_move(10, 0) == LIFT_COMMAND_UNCONFIGURED);
    assert(app_lift_configure(NAN, 100) == LIFT_COMMAND_INVALID);
    assert(app_lift_configure(10.0f, 100) == LIFT_COMMAND_OK);
    assert(app_lift_move(10, 0) == LIFT_COMMAND_NEEDS_ZERO);

    /* 零点靠近 32 位回绕点，正常上升会跨过它。 */
    uint32_t zero = UINT32_MAX - 25U;
    encoder_count = zero;
    assert(app_lift_set_zero() == LIFT_COMMAND_OK);
    assert(app_lift_move(101, 0) == LIFT_COMMAND_INVALID);
    assert(app_lift_move(20, 0) == LIFT_COMMAND_OK);
    app_lift_update(10);
    assert(output_permille == LIFT_RUN_PWM_PERMILLE);
    assert(app_lift_set_zero() == LIFT_COMMAND_BUSY);
    assert(app_lift_move(10, 10) == LIFT_COMMAND_BUSY);

    encoder_count = zero + 175U;
    app_lift_update(100);
    assert(output_permille == LIFT_APPROACH_PWM_PERMILLE);
    encoder_count = zero + 196U;
    app_lift_update(200);
    assert(output_permille == 0);
    assert(app_lift_get_status().state == LIFT_AT_TARGET);

    assert(app_lift_move(0, 210) == LIFT_COMMAND_OK);
    app_lift_update(220);
    assert(output_permille == -LIFT_RUN_PWM_PERMILLE);
    encoder_count = zero;
    app_lift_update(300);
    assert(output_permille == 0);
    assert(app_lift_get_status().position_mm == 0.0f);
}

/** 验证无进展、反向反馈、越界和通信停机。 */
static void test_faults(void) {
    assert(app_lift_move(10, 500) == LIFT_COMMAND_OK);
    app_lift_update(500 + LIFT_PROGRESS_TIMEOUT_MS);
    lift_status_t status = app_lift_get_status();
    assert(status.fault == LIFT_FAULT_NO_PROGRESS && !status.referenced);
    assert(output_permille == 0);

    assert(app_lift_set_zero() == LIFT_COMMAND_OK);
    uint32_t zero = encoder_count;
    encoder_count = zero + 20U;
    assert(app_lift_move(10, 1000) == LIFT_COMMAND_OK);
    encoder_count -= 2U;
    app_lift_update(1010);
    assert(app_lift_get_status().fault == LIFT_FAULT_DIRECTION);
    assert(output_permille == 0);

    encoder_count = zero;
    assert(app_lift_set_zero() == LIFT_COMMAND_OK);
    assert(app_lift_move(10, 1100) == LIFT_COMMAND_OK);
    encoder_count = zero + 1001U;
    app_lift_update(1110);
    assert(app_lift_get_status().fault == LIFT_FAULT_RANGE);
    assert(output_permille == 0);

    encoder_count = zero;
    assert(app_lift_set_zero() == LIFT_COMMAND_OK);
    assert(app_lift_move(10, 1200) == LIFT_COMMAND_OK);
    app_lift_update(1210);
    app_lift_stop(LIFT_FAULT_LINK_LOST);
    app_lift_update(1220);
    assert(app_lift_get_status().fault == LIFT_FAULT_LINK_LOST);
    assert(output_permille == 0);

    assert(app_lift_move(10, 1300) == LIFT_COMMAND_OK);
    encoder_count = zero + 110U;
    app_lift_update(1310);
    assert(app_lift_get_status().fault == LIFT_FAULT_OVERSHOOT);
    assert(output_permille == 0);
}

/** 验证 C 解析器收到大端标定指令，并拒绝 CRC 或长度错误。 */
static void test_protocol(void) {
    proto_t parser;
    const proto_cb_t callbacks = {.on_lift_command = capture_command};
    proto_init(&parser, &callbacks);

    /* 序号 1，标定 10 个计数/mm，最大行程 100 mm。 */
    uint8_t frame[] = {0xAA, 0x55, 1, CMD_LIFT_CONFIG, 6, 0x41, 0x20, 0, 0, 0, 100, 0, 0};
    uint16_t crc = proto_crc16(&frame[2], 9);
    frame[11] = (uint8_t)(crc >> 8);
    frame[12] = (uint8_t)crc;
    for (unsigned i = 0; i < sizeof(frame); ++i) {
        proto_feed(&parser, frame[i]);
    }
    assert(parser.rx_frames == 1 && received_command == CMD_LIFT_CONFIG);
    assert(received_scale == 10.0f && received_height == 100);

    frame[12] ^= 1;
    for (unsigned i = 0; i < sizeof(frame); ++i) {
        proto_feed(&parser, frame[i]);
    }
    assert(parser.rx_frames == 1 && parser.crc_errors == 1);

    uint8_t malformed[] = {0xAA, 0x55, 1, CMD_LIFT_MOVE, 0, 0, 0};
    crc = proto_crc16(&malformed[2], 3);
    malformed[5] = (uint8_t)(crc >> 8);
    malformed[6] = (uint8_t)crc;
    for (unsigned i = 0; i < sizeof(malformed); ++i) {
        proto_feed(&parser, malformed[i]);
    }
    assert(parser.rx_frames == 1 && received_command == CMD_LIFT_CONFIG);
}

int main(void) {
    test_positioning();
    test_faults();
    test_protocol();
    return 0;
}
