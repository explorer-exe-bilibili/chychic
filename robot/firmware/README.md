# 下位机固件（STM32）

`App/` 里是**可以直接拖进 CubeMX 工程**的应用层代码，不碰 `Core/`。
这样 CubeMX 重新生成代码不会覆盖你的逻辑。

```
firmware/App/Inc/protocol.h   App/Src/protocol.c   协议编解码 + CRC16
firmware/App/Inc/motor.h      App/Src/motor.c      双轮驱动 + 编码器测速 + 速度 PI
firmware/App/Inc/app.h        App/Src/app.c        看门狗 + 100Hz 上报 + 回调串联
```

## 一、CubeMX 要配的外设

| 外设 | 配置 | 用途 |
|---|---|---|
| `TIM1` CH1/CH2 | PWM Generation, ARR=1000, 20kHz | 左右电机 PWM |
| `TIM2` | **Encoder Mode**, TI1&TI2, 4 倍频 | 左编码器 |
| `TIM3` | Encoder Mode, TI1&TI2, 4 倍频 | 右编码器 |
| `USART1` | 115200 8N1, 开全局中断 | 和上位机通信 |
| `TIM6` | 1ms 周期中断 | 控制节拍 |
| GPIO | PB0–PB3 推挽输出 | L298N 的 IN1–IN4 |
| GPIO | PA0–PA4 上拉输入 | 五路红外 |

> **编码器 4 倍频**：CubeMX 里 `Encoder Mode` 选 `TI1 and TI2` 就是 4 倍频，
> 计数分辨率最高。如果电机是霍尔编码器（11 线），减速比 1:30，
> 轮子转一圈 = `11 × 4 × 30 = 1320` 计数。

## 二、接进工程（三处改动）

**1. `main.c` 的 `/* USER CODE BEGIN 2 */`**

```c
#include "app.h"
app_init();
HAL_TIM_Base_Start_IT(&htim6);
```

**2. `main.c` 末尾的 `/* USER CODE BEGIN 4 */`**

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        app_on_uart_rx(app_rx_byte);   /* app_rx_byte 由 app.h 声明 */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
        app_tick_1khz();
}
```

**3. `app.c` 顶部改配置**

```c
#define APP_WHEEL_BASE      0.150f   /* 你车的轮距，米 */
#define APP_TICKS_PER_METER 6000.0f  /* 见下面的标定 */
```

再按实际接线改 `s_ir[]` 数组（红外）、`s_motor.in*_pin`（方向脚）、
`s_motor.htim_*`（定时器句柄）。

## 三、`ticks_per_meter` 标定（**必做，不标定速度环就是错的**）

1. 上位机发 `MODE_CALIB`（电机不转，只累计编码器）
2. 手推车沿直线走**正好 1 米**（地上贴胶带做记号）
3. 读上位机日志里的 `enc_l`（或 `enc_r`）末值
4. `ticks_per_meter = 那个数 / 1.0`

两次取平均，再除以实测轮径修正一次。这个值差 20%，速度环就会震荡。

## 四、上电与安全

- 上电默认 `MODE_IDLE`，**电机断使能**，必须上位机显式发 `MODE_RUN` 才动
- 上位机连续 **200ms** 没有合法帧 → 自动刹车并置 `ST_WATCHDOG`
- `CMD_ESTOP` 后必须显式发 `MODE_IDLE` 才解除，防止误恢复
- **烧录时把车架起来**，轮子离地

## 五、常见坑

| 现象 | 原因 |
|---|---|
| 上位机一直收到 CRC 错 | 两端 CRC 参数不一致（poly/init），或者大小端反了 |
| 编码器数只增不减 | 编码器模式没设 4 倍频，或者 A/B 相接反（换线序即可） |
| 速度环啸叫 | `kp` 太大 / 控制周期不是 1ms |
| 车原地抽 | 积分项没限幅，或者 `target=0` 时占空比没归零 |
| 上位机一挂车就冲 | 看门狗没生效，检查 `APP_WDOG_TIMEOUT_MS` 那段 |

## 六、先验证协议，再上电机

上位机侧有自测脚本，**先跑通协议再加电**：

```bash
cd ../host && python3 tools/test_protocol.py
```

它打印一条金帧十六进制，让写固件的同学在串口助手里逐字节核对。
两端 CRC 对不上，后面所有调试都是浪费时间。
