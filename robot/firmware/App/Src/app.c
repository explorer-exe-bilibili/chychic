/**
 * @file    app.c
 */
#include "app.h"
#include "protocol.h"
#include "motor.h"

#include <string.h>

/* ========================= 需要按实际接线改的配置 =======================
 * 这些值全部集中在这里，改的时候不用满工程找。
 * ======================================================================= */
#define APP_WHEEL_BASE      0.150f   /* 轮距 m */
#define APP_TICKS_PER_METER 6000.0f  /* 轮子转 1m 的编码器计数，必须实测 */
#define APP_WDOG_TIMEOUT_MS 200u     /* 200ms 没收到上位机指令就刹车 */
#define APP_FB_PERIOD_MS    10u      /* 100Hz 上报 */

extern UART_HandleTypeDef huart1;    /* 上位机通信口 */

static proto_t  s_proto;
static motor_t  s_motor;
static uint32_t s_tick_ms;
static uint32_t s_last_rx_ms;
static uint8_t  s_mode;
static uint8_t  s_status;
static uint8_t  s_tx_seq;
uint8_t         app_rx_byte;          /* 给 HAL_UART_RxCpltCallback 用 */
static uint8_t  s_tx_buf[40];

/* ============================ 传感器 =================================== */

/* 五路红外：从左到右，1 = 压线。按自己的引脚改。 */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       invert;   /* 模块输出低电平有效就置 1 */
} ir_pin_t;

static const ir_pin_t s_ir[5] = {
    {GPIOA, GPIO_PIN_0, 1},
    {GPIOA, GPIO_PIN_1, 1},
    {GPIOA, GPIO_PIN_2, 1},
    {GPIOA, GPIO_PIN_3, 1},
    {GPIOA, GPIO_PIN_4, 1},
};

static uint8_t read_ir_word(void)
{
    uint8_t word = 0;
    for (uint8_t i = 0; i < 5u; i++)
    {
        GPIO_PinState st = HAL_GPIO_ReadPin(s_ir[i].port, s_ir[i].pin);
        uint8_t high = (st == GPIO_PIN_SET) ? 1u : 0u;
        uint8_t active = s_ir[i].invert ? (uint8_t)!high : high;
        if (active)
            word |= (uint8_t)(1u << i);
    }
    return word;
}

/* ============================ 协议回调 ================================= */

static void on_vel(float v, float w)
{
    s_last_rx_ms = s_tick_ms;
    /* 只有 RUN 才执行运动，IDLE/ESTOP/CALIB 一律忽略速度指令 */
    if (s_mode == MODE_RUN)
        motor_set_twist(&s_motor, v, w, APP_WHEEL_BASE);
}

static void on_mode(uint8_t mode)
{
    s_last_rx_ms = s_tick_ms;
    if (mode == s_mode)
        return;

    s_mode = mode;
    switch (mode)
    {
    case MODE_RUN:
        s_status &= (uint8_t)~ST_ESTOP;
        break;
    case MODE_ESTOP:
        motor_brake(&s_motor);
        s_status |= ST_ESTOP;
        break;
    case MODE_CALIB:
        motor_set_calib_mode(&s_motor);
        break;
    case MODE_IDLE:
    default:
        motor_brake(&s_motor);
        s_status &= (uint8_t)~ST_ESTOP;
        break;
    }
}

static void on_estop(void)
{
    s_last_rx_ms = s_tick_ms;
    s_mode = MODE_ESTOP;
    motor_brake(&s_motor);
    s_status |= ST_ESTOP;
}

/* ============================ 初始化 =================================== */

void app_init(void)
{
    proto_cb_t cb = {on_vel, on_mode, on_estop};
    proto_init(&s_proto, &cb);

    memset(&s_motor, 0, sizeof(s_motor));
    s_motor.htim_pwm    = &htim1;          /* 改成自己的 PWM 定时器 */
    s_motor.ch_l        = TIM_CHANNEL_1;
    s_motor.ch_r        = TIM_CHANNEL_2;
    s_motor.in1_port    = GPIOB; s_motor.in1_pin = GPIO_PIN_0;
    s_motor.in2_port    = GPIOB; s_motor.in2_pin = GPIO_PIN_1;
    s_motor.in3_port    = GPIOB; s_motor.in3_pin = GPIO_PIN_2;
    s_motor.in4_port    = GPIOB; s_motor.in4_pin = GPIO_PIN_3;
    s_motor.htim_enc_l  = &htim2;          /* 编码器模式 */
    s_motor.htim_enc_r  = &htim3;
    s_motor.ticks_per_meter = APP_TICKS_PER_METER;
    s_motor.pwm_max     = (float)__HAL_TIM_GET_AUTORELOAD(&htim1);
    motor_init(&s_motor);

    s_mode = MODE_IDLE;                    /* 上电默认 IDLE，绝不默认 RUN */
    s_status = ST_ENCODER;
    s_last_rx_ms = 0;
    s_tick_ms = 0;
    s_tx_seq = 0;

    /* 先把接收中断武装起来 */
    HAL_UART_Receive_IT(&huart1, &app_rx_byte, 1);
}

/* ============================ 接收 ===================================== */

void app_on_uart_rx(uint8_t byte)
{
    proto_feed(&s_proto, byte);
    HAL_UART_Receive_IT(&huart1, &app_rx_byte, 1);   /* 重新武装，别忘 */
}

/* ============================ 周期任务 ================================= */

void app_tick_1khz(void)
{
    s_tick_ms++;

    /* 1) 看门狗：上位机挂了就刹车。这是整套代码里最重要的一行逻辑。 */
    if (s_mode == MODE_RUN &&
        (uint32_t)(s_tick_ms - s_last_rx_ms) > APP_WDOG_TIMEOUT_MS)
    {
        motor_brake(&s_motor);
        s_mode = MODE_IDLE;
        s_status |= ST_WATCHDOG;
    }

    /* 2) 速度环 */
    motor_update(&s_motor, 0.001f);

    /* 3) 100Hz 上报 */
    if ((s_tick_ms % APP_FB_PERIOD_MS) == 0u)
    {
        uint16_t n = proto_build_state(
            s_tx_buf, s_tx_seq++,
            s_tick_ms,
            s_motor.meas_l, s_motor.meas_r,
            s_motor.enc_l, s_motor.enc_r,
            read_ir_word(),
            s_status);

        /* 忙就丢这一帧，绝不阻塞 1kHz 中断 */
        if (huart1.gState == HAL_UART_STATE_READY)
            HAL_UART_Transmit_IT(&huart1, s_tx_buf, n);
    }
}
