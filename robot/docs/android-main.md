# 用 Android 手机当上位机（临时方案）

> 适用场景：树莓派暂时拿不到，用一台已 root 的旧手机顶几周。
> **不是推荐方案，是没得选时的方案。** 风险和代价写在最后一节。

## 0. 为什么不能像树莓派那样直接插

被测机型（vivo V2072A，MTK Dimensity 1100，Android 13，内核 4.19.191）：

```
# CONFIG_USB_SERIAL is not set      → CH340 / CP2102 全都不认
# CONFIG_USB_ACM is not set         → STM32 原生 USB CDC 也认不到
# CONFIG_MEDIA_USB_SUPPORT is not set → UVC 摄像头也没有 /dev/video*
```

CH340 插上能枚举（`1a86:7523`），但**没有任何驱动绑定它**，`/dev/ttyUSB0` 不会出现。
内核不可改（没有厂商源码），所以只能在**用户态**实现串口协议 —— 这正是
`usb-serial-for-android` 在 Java 里做的事，我们用 Python + pyusb 做同样的事。

前提是 usbfs 对 root 开放。实测这台机器可以：

```bash
su -c 'cat /dev/bus/usb/001/004'   # rc=0，能读出设备描述符
```

## 1. 装环境

在手机上装 **Termux**（F-Droid 版，别用 Google Play 那个停更版），然后：

```bash
pkg update && pkg upgrade -y
pkg install python libusb root-repo -y
pip install pyusb
```

> Termux 的 `su` 依赖 Magisk / KernelSU / APatch。这台机器三个都装了，
> `su -c id` 返回 `uid=0(root) context=u:r:magisk:s0`。

## 2. 验证串口（先做这个，别急着跑算法）

把 CH340 的 **TX 和 RX 短接**，做回环：

```bash
su -c "python ~/android_ch340.py --selftest"
```

期望输出：

```
✓ 已打开，芯片版本 0x27
  分频值 0xCC03  @ 115200 bps
  写入 16 字节: 20 21 22 ...
  读回 16 字节: 20 21 22 ...
✓ 回环通过，用户态串口可用
```

看到 `✓` 才继续。失败先看这三项：OTG 线是否走数据、是否 root 运行、`lsusb` 能否看到 `1a86:7523`。

## 3. 接真车

```bash
su -c "python main.py --port android --camera 'http://127.0.0.1:8080/video'"
```

`--port android` 会走 `android_ch340.Ch340`，主循环其余部分和树莓派上完全一样。

## 4. 摄像头

内置摄像头由 App 在本机开 MJPEG 服务（如 IP Webcam），OpenCV 直接读 HTTP 地址。
USB 摄像头（`0bda:3031`，标准 UVC）虽然能枚举，但内核没有 UVC 驱动，
Python 拿不到 —— 除非自己实现用户态 UVC（等时传输，工作量大，不建议）。

**代价是延迟**：MJPEG 经 App 转发，典型 60–120 ms。所以：

> **循迹 PID 必须放在 STM32 上跑**，手机只做 20 Hz 的高层决策。
> 这不是妥协，对上树莓派也有利。

## 5. 风险清单（决定要不要用之前先读完）

| 风险 | 影响 | 缓解 |
|---|---|---|
| 用户态 USB 驱动没在比赛中验证过 | 掉一次线丢一次发车 | 赛前连续跑 20 轮做压力测试 |
| MJPEG 延迟 60–120 ms | 高层决策滞后 | PID 下放；降速；缩小分辨率 |
| Termux + root 进程被系统杀 | 跑一半没了 | 关电池优化、加白名单 |
| 手机电池 4000+ mAh | S11 写着"单个电池 ≤3000mAh" | 规则大概率指主电池，但**必须去答疑确认** |
| 供电：手机自己耗电 | 一轮几十分钟可能撑不住 | 接充电宝，注意整机重量 ≤10 kg |

**如果这周还来得及买到一块 ESP32（约 10 元）**，用它做 UART↔Wi-Fi 桥会比这套稳得多：
手机端只要 `socket.sendto` 二十行，不用碰 librusb，也不用 root。
本方案的价值在于**零成本、今天就能验证**。

## 6. 相关文件

| 文件 | 作用 |
|---|---|
| `host/android_ch340.py` | 用户态 CH340 驱动（寄存器序列对应内核 `ch341.c`） |
| `host/link.py` | `--port android` 走这条通路，其余逻辑不变 |
| `host/vision/camera.py` | `--camera` 支持 URL，读 MJPEG 流 |
