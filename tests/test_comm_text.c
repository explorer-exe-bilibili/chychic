#include "app_comm_text.h"
#include "protocol.h"

#include <assert.h>
#include <string.h>

/** 将 data 中的 length 个字节交给实际解析器，返回收齐的最后一条文本命令。 */
static comm_text_command_t feed(comm_text_rx_t *rx, proto_t *binary, const void *data,
                                size_t length) {
    const uint8_t *bytes = data;
    comm_text_command_t command = {0};
    for (size_t index = 0; index < length; ++index) {
        char *line;
        if (app_comm_text_feed(rx, bytes[index], binary->state == PS_SOF1, &line)) {
            if (line != NULL) {
                command = app_comm_text_parse(line);
            }
        } else {
            proto_feed(binary, bytes[index]);
        }
    }
    return command;
}

/** 验证网页所用的命令、分片、结束符和错误参数。 */
static void test_commands(void) {
    comm_text_rx_t rx = {0};
    proto_t binary;
    proto_init(&binary, NULL);
    const char first[] = "!pl";
    const char last[] = "ot on#";
    assert(feed(&rx, &binary, first, strlen(first)).action == COMM_TEXT_INVALID);
    assert(feed(&rx, &binary, last, strlen(last)).action == COMM_TEXT_PLOT_ON);

    const char pid[] = "!pid both 0.25 0.1 0\r\n";
    comm_text_command_t command = feed(&rx, &binary, pid, strlen(pid));
    assert(command.action == COMM_TEXT_PID_SET && command.wheel == MOTOR_COUNT);
    assert(command.gains.kp == 0.25f && command.gains.ki == 0.1f && command.gains.kd == 0.0f);

    const char invalid[] = "!pid left nan 0 0#";
    assert(feed(&rx, &binary, invalid, strlen(invalid)).action == COMM_TEXT_INVALID);
    const char speed[] = "!speed -100#";
    command = feed(&rx, &binary, speed, strlen(speed));
    assert(command.action == COMM_TEXT_SPEED && command.speed_mm_s == -100.0f);
    const char idle[] = "!idle#";
    assert(feed(&rx, &binary, idle, strlen(idle)).action == COMM_TEXT_IDLE);
}

/** 验证二进制有效载荷里的文本命令不会被执行，并能打断未完成的文本命令。 */
static void test_binary_interleaving(void) {
    comm_text_rx_t rx = {0};
    proto_t binary;
    proto_init(&binary, NULL);
    uint8_t frame[] = {0xAA, 0x55, 0x21, CMD_VEL, 8, '!', 'i', 'd', 'l', 'e', '#', 0, 0, 0, 0};
    uint16_t crc = proto_crc16(&frame[2], 11);
    frame[13] = (uint8_t)(crc >> 8);
    frame[14] = (uint8_t)crc;
    const char incomplete[] = "!speed ";
    (void)feed(&rx, &binary, incomplete, strlen(incomplete));
    assert(feed(&rx, &binary, frame, sizeof(frame)).action == COMM_TEXT_INVALID);
    assert(binary.rx_frames == 1U);
    const char ping[] = "!ping#";
    assert(feed(&rx, &binary, ping, strlen(ping)).action == COMM_TEXT_PING);
}

/** 验证每组数据有四个命名通道，负轮速原样保留。 */
static void test_plot_format(void) {
    char line[100];
    size_t length = app_comm_text_plot(line, sizeof(line), 100.0f, 92.5f, -100.0f, -94.25f);
    assert(length == strlen(line));
    assert(strcmp(line, "left_target=100.00,left_actual=92.50,"
                        "right_target=-100.00,right_actual=-94.25\n") == 0);
    assert(app_comm_text_plot(line, 10, 100.0f, 92.5f, -100.0f, -94.25f) == 0);
}

int main(void) {
    test_commands();
    test_binary_interleaving();
    test_plot_format();
    return 0;
}
