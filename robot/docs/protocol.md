# 上下位机通信协议 v1.0

> 上位机（ARM64 Linux / 树莓派 / RK）= 算法，下位机（STM32）= 电控。
> 参考实现：`host/protocol.py` ↔ `firmware/App/Src/protocol.c`，两端 CRC 必须一致。

## 1. 物理层

| 项 | 值 |
|---|---|
| 接口 | UART / USB-CDC 均可 |
| 波特率 | 115200 8N1（够用；要更高就 921600，两端一起改） |
| 连线 | TX↔RX 交叉，共地 |

## 2. 帧格式（大端）

```
+------+------+-----+-----+-----+-----------+--------+--------+
| 0xAA | 0x55 | SEQ | CMD | LEN | PAYLOAD…  | CRC_H  | CRC_L  |
+------+------+-----+-----+-----+-----------+--------+--------+
```

- `SEQ`：发送方自增序号，0–255 回绕，用于丢帧统计
- `CMD`：消息 ID
- `LEN`：payload 字节数，上限 64
- `CRC16`：**CCITT-FALSE**（poly=`0x1021`，init=`0xFFFF`），覆盖 `SEQ | CMD | LEN | PAYLOAD`

## 3. 消息定义

### 上位机 → 下位机

| CMD | 名称 | 频率 | Payload |
|---|---|---|---|
| `0x10` | `CMD_VEL` | 仅 MANUAL 模式，50 Hz | `<float32 v_mm_s, float32 w_rad_s>` |
| `0x11` | `CMD_MODE` | 变化时 + 1 Hz 重发 | `<uint8 mode>` |
| `0x12` | `CMD_ESTOP` | 事件 | 空 |
| `0x13` | `CMD_PING` | **20 Hz，常发** | 空 |

`CMD_VEL` 的单位是 **mm/s**（对齐 MCU 的 `chassis_set_velocity`），
上位机内部用 m/s，换算只在 `link.SerialLink.send_vel` 一处做。

`mode`：

| 值 | 含义 | 行为 |
|---|---|---|
| 0 | `IDLE` | 电机断使能，可手推 |
| 1 | `LINE_FOLLOW` | **MCU 自主循迹**，忽略 `CMD_VEL` |
| 2 | `MANUAL` | 听 `CMD_VEL`，用于 A 区绕行 / C 区对中 |
| 3 | `ESTOP` | 立即刹车并锁死，必须显式切回 `IDLE` 才解除 |

> **为什么 `CMD_PING` 要常发**：循迹虽然由 MCU 自己做，但上位机一死，
> 车就成了没人管的自主小车。看门狗 300 ms 无帧 → 停车。所以 LINE_FOLLOW
> 模式下也必须以 20 Hz 续命。

### 下位机 → 上位机

| CMD | 名称 | 频率 | Payload（大端，共 36 字节） |
|---|---|---|---|
| `0x80` | `FB_STATE` | 20–50 Hz | 见下 |
| `0x81` | `FB_ACK` | 收到即回 | `<uint8 acked_cmd>` |

```
偏移  类型      字段            说明
  0   uint32   tick_ms         下位机上电毫秒数
  4   uint8    mode            当前模式（同上表）
  5   uint8    lf_state        line_follow_state_t
  6   uint8    line_mask       五路归一化位图，bit0 最左，1=黑线
  7   float32  lf_error        位置偏差，左正右负，范围 [-2, 2]
 11   float32  v_cmd           车体线速度目标 mm/s
 15   float32  w_cmd           车体角速度目标 rad/s
 19   float32  v_l             左轮实际 mm/s
 23   float32  v_r             右轮实际 mm/s
 27   int32    enc_l           左轮累计计数
 31   int32    enc_r           右轮累计计数
 35   uint8    status          位标志
```

`lf_state`：

| 值 | 含义 |
|---|---|
| 0 | 未启用 |
| 1 | 找线中（ACQUIRING） |
| 2 | 循迹中（TRACKING） |
| 3 | 丢线（LOST） |
| 4 | 宽线（WIDE_LINE，可能是路口） |
| 5 | 多线段（AMBIGUOUS） |

`status` 位定义：

| bit | 含义 |
|---|---|
| 0 | 编码器已就绪 |
| 1 | 低电压 |
| 2 | 看门狗超时（最近 300ms 没收到上位机帧） |
| 3 | 急停生效 |
| 4 | 电机堵转 |

## 4. 安全规则（务必实现）

1. **看门狗**：下位机连续 **300 ms** 没收到合法帧 → 电机自动停转，`status.bit2` 置位。
   上位机必须以 20 Hz 发 `CMD_PING`，`LINE_FOLLOW` 模式下也不例外。
2. **急停优先**：`CMD_ESTOP` 立即生效，且必须显式下发 `CMD_MODE=IDLE` 才解除，防止误恢复。
3. **CRC 错误丢帧**，不影响后续解析（解析器要能重新同步到 `0xAA 0x55`）。
4. **上电默认 IDLE**，绝不默认 RUN。

## 5. 为什么这么设计

- **固定帧长 + 校验**：比赛现场电机干扰大，纯文本协议（`$V,0.3#`）容易解析错位。
- **100 Hz 上行**：算法侧做视觉时也要知道轮速，反馈慢了融合就没意义。
- **`FB_STATE` 里带 `enc_l/enc_r`**：里程计算在上位机做，方便以后换算法。
- **200 ms 看门狗**：视觉进程崩了，车自己停，不至于冲出场。
