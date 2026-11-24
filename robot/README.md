# 智竞杯 · 算法骨架

针对 **第二届"智竞杯"东华大学机器人大赛 / RoboMaster 2027 机甲大师校内赛** 的算法侧起始工程。
上位机 ARM64 Linux（树莓派 / RK3588）跑视觉与决策，下位机 STM32 跑电机闭环与传感器采集。

> 这不是能直接拿冠军的成品，是一个**正确的起点**：协议、状态机、日志、安全兜底都在位了，
> 你们往里填视觉和抓取逻辑就行，不用从零搭架子。

## 5 分钟跑起来（不用任何硬件）

```bash
cd host
pip install pyserial numpy          # 视觉部分另见 requirements.txt
python3 tools/test_protocol.py      # 先验协议，9 项全 PASS
python3 main.py --sim --no-vision --no-qr --duration 5
```

跑完 `logs/` 里会多一份 CSV 和一份事件日志，这就是后面每次调参的档案。
仿真链路会模拟红外和里程，用来验证**状态机 + 日志 + 控制链路**是否正常，
不代表实际控制效果。

## 接上真车

```bash
python3 main.py --port /dev/ttyUSB0          # 正常
python3 main.py --port /dev/ttyUSB0 --video  # 顺便录像
python3 main.py --port /dev/ttyUSB0 --no-vision   # 只跑红外，排查相机
```

键盘：`空格` 急停，`q` 退出。

## 手动遥控（建议先跑这个）

在车上跑起来，笔记本浏览器打开就能开：

```bash
python3 teleop.py --port /dev/ttyUSB0     # 手机用 --port android
# 浏览器打开 http://<车上设备IP>:8000
```

拖摇杆或按 WASD。好处是**不依赖任何第三方库**，而且能同时验证三件事：
串口链路、电机转向、速度环。转向反了就在 `chassis_config.h` 里改极性，
不用等算法。

安全设计：浏览器 20Hz 上报，**超过 400ms 没有指令自动停车**——拔网线、
关笔记本、切 Wi-Fi 都不会失控。服务端还有一层限幅，前端改参数也超不过
`--v-max / --w-max`。

## 目录

```
docs/protocol.md          通信协议（帧格式、消息、安全规则）
host/
  main.py                 主循环 50Hz
  params.py               **全车硬件参数，实测后只改这里**
  protocol.py             编解码 + CRC16（与固件逐位一致）
  link.py                 串口链路 / 仿真链路
  odometry.py             编码器 -> 位姿（差速航迹推算）
  line_follow.py          红外 + 视觉融合的循迹控制器
  pid.py                  PID + 限速器
  fsm.py                  任务状态机（A→B→C→D）
  logger.py               CSV + 事件 + 录像
  vision/line.py          赛道黑线检测
  vision/qr.py            启动二维码解析
  vision/camera.py        独立线程抓帧（防卡死）
  teleop.py               网页遥控：浏览器当手柄，先验底盘
  tools/test_protocol.py  协议自测（不用硬件）
  tools/bringup.py        上车标定：方向 / ticks_per_meter / 轮距
  android_ch340.py        Android 用户态 CH340 驱动（手机当主控时用，见 docs/android-main.md）
firmware/
  App/…                   STM32 应用层（协议 / 电机 / 看门狗）
  README.md               接入 CubeMX 的三处改动 + 标定步骤
```

## 数据流

```
相机 ──► LineDetector ──► offset/conf ─┐
                                        ├─► LineFollower ──► (v, w) ──► 串口 ──► STM32
五路红外 ◄──────────────────────────────┘                                     │
                                                                            │
        └──────────────── FB_STATE (100Hz) ◄──────────────────────────────────┘
```

## 状态机和规则的对应

| 状态 | 对应任务 | 分值 | 现在实现到哪 |
|---|---|---|---|
| `WAIT_QR` | 扫二维码领任务 | 5 | ✅ 识别 + 解析 |
| `SEG_A` | 基础任务（直线/圆弧/直角） | 25 | ✅ 循迹控制器 |
| `SEG_A` | A 区智能避障 | 20 | ⬜ 待填颜色识别 + 推/绕决策 |
| `SEG_B` | B 区精细抓取 | ~20 | ⬜ 占位：目前直接循迹穿过 |
| `SEG_C` | C 区精细放置 | ~20 | ⬜ 占位：目前直接循迹穿过 |
| `SEG_D` | D 区曲线循迹 | 10 | ✅ 循迹控制器（陡弯需单独调参） |
| `RETURN` | 回到启停区 | — | ✅ |

**当前这套代码能直接支撑的是「基础 30 分 + D 区 10 分」这一档**，
也就是先把最容易的 40 分焊死。A 区、B 区、C 区是留给你们的填空题。

## 立刻要做的三件事

1. **上车标定**（一条命令走完，输出直接抄进 `params.py`）：

   ```bash
   cd host
   python3 tools/bringup.py --port /dev/ttyUSB0
   ```

   依次验：能不能收到状态 → 编码器方向对不对 → 推车一米算 `TICKS_PER_METER`
   → 原地转一圈算 `WHEEL_BASE`。**第一次跑把车架起来**，3、4 步再落地。

2. **对拍协议**：`tools/test_protocol.py` 打的金帧，让下位机逐字节核对。
   两端 CRC 对不上时不要往下做，浪费时间。

3. **实测调 `LineConfig`**（`host/line_follow.py` 顶部）：
   `v_cruise` 从 0.25 开始，`kp` 从 1.0 往上加，直到直道能压住线又不抖。

## 还没做的部分（避免误用）

- **A 区避障**：颜色识别、对准、横推、回线
- **B/C 区**：物料识别、抓取位姿、圆环定位、放置伺服
- **AprilTag 定位**：规则允许每队最多 6 个，D 区禁贴
- **麦轮平移**：`motor_set_twist` 目前只做差速，要平移得扩协议

## 一个必须去问组委会的问题

`vision/qr.py` 里把二维码第二段的解析做成了**可切换常量**（`RING_ORDER`）。
因为规则手册的例子和文字描述互相矛盾：

> 「123+123 意为……在 C 区把绿色放 **3** 号圆环、深蓝放 **2** 号、浅蓝放 **1** 号」

按位置对应应该是 绿→1、深蓝→2、浅蓝→3，文字却是反的。
拿到官方答复后改这一个常量即可，其他代码不用动。
