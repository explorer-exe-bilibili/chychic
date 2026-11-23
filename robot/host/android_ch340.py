"""Android 用户态 CH340 串口驱动（pyusb + libusb）。

## 为什么需要这个

这台手机（vivo V2072A，MTK 4.19.191）内核里：

    # CONFIG_USB_SERIAL is not set
    # CONFIG_USB_ACM is not set
    # CONFIG_MEDIA_USB_SUPPORT is not set

所以 CH340 插上去能枚举（`1a86:7523`），但**没有任何驱动去绑定它**，
`/dev/ttyUSB0` 不会出现。

好消息是 usbfs 对 root 是通的 —— `su -c 'cat /dev/bus/usb/001/004'` 能读出
设备描述符。所以可以像 `usb-serial-for-android` 那样，**在用户态直接实现
CH340 协议**，完全绕过内核驱动。

## 协议来源

寄存器序列与分频算法逐行对应 Linux 内核 `drivers/usb/serial/ch341.c`：

    CH341_REQ_READ_VERSION 0x5F  读芯片版本（期望 0x27 或 0x30）
    CH341_REQ_SERIAL_INIT  0xA1  串口初始化
    CH341_REQ_WRITE_REG    0x9A  写寄存器
    CH341_REQ_MODEM_CTRL   0xA4  DTR/RTS

    get_divisor(): 48MHz / (clk_div * speed)，带四舍五入与 fact 折半优化

## 用法

    sudo python3 android_ch340.py --selftest     # 需要 TX-RX 短接做回环
    sudo python3 android_ch340.py --monitor      # 十六进制打印收到的字节

在 Termux 里必须以 root 跑（usbfs 需要）：

    su -c "python ~/ch340/android_ch340.py --selftest"
"""

from __future__ import annotations

import argparse
import sys
import time

VID = 0x1A86
PID = 0x7523

# CH341 请求码
REQ_READ_VERSION = 0x5F
REQ_WRITE_REG = 0x9A
REQ_READ_REG = 0x95
REQ_SERIAL_INIT = 0xA1
REQ_MODEM_CTRL = 0xA4

# 寄存器地址
REG_BREAK = 0x05
REG_PRESCALER = 0x12
REG_DIVISOR = 0x13
REG_LCR = 0x18
REG_LCR2 = 0x25
REG_FLOW_CTL = 0x27

# 线路控制：8 数据位 + 使能收发 = 8N1 的基础值
LCR_ENABLE_RX = 0x80
LCR_ENABLE_TX = 0x40
LCR_CS8 = 0x03
LCR_8N1 = LCR_ENABLE_RX | LCR_ENABLE_TX | LCR_CS8  # 0xC3

# 调制解调器控制位
BIT_RTS = 1 << 6
BIT_DTR = 1 << 5

CLKRATE = 48_000_000
MIN_BPS = 46          # ceil(48e6 / (4096 * 256))
MAX_BPS = 3_000_000   # 48e6 / (8 * 2)

DEFAULT_TIMEOUT_MS = 1000


def _clk_div(ps: int, fact: int) -> int:
    """对应内核宏 CH341_CLK_DIV(ps, fact)。"""
    return 1 << (12 - 3 * ps - fact)


# 各预分频档位支持的最低波特率（对应 ch341_min_rates[]）
_MIN_RATES = [CLKRATE // (_clk_div(ps, 1) * 512) for ps in range(4)]


def get_divisor(speed: int) -> int:
    """波特率 -> 写入 (REG_DIVISOR, REG_PRESCALER) 的 16 位值。

    逐行对应内核 ch341_get_divisor()，包括末尾那两处取整修正。
    115200 的标准结果是 0xCC03（fact=0, ps=3, div=52）。
    """
    speed = max(MIN_BPS, min(MAX_BPS, speed))

    ps = -1
    for p in (3, 2, 1, 0):
        if speed > _MIN_RATES[p]:
            ps = p
            break
    if ps < 0:
        raise ValueError(f"不支持的波特率 {speed}")

    fact = 1
    cd = _clk_div(ps, fact)
    div = CLKRATE // (cd * speed)

    # 分频值超范围就折半基频
    if div < 9 or div > 255:
        div //= 2
        cd *= 2
        fact = 0
    if div < 2:
        raise ValueError(f"波特率 {speed} 无法精确分频")

    # 选更接近目标值的那一档（放大 16 倍避免整数误差）
    hi = 16 * CLKRATE // (cd * div) - 16 * speed
    lo = 16 * speed - 16 * CLKRATE // (cd * (div + 1))
    if hi >= lo:
        div += 1

    # 偶数分频优先走低基频，接收端容错更好
    if fact == 1 and div % 2 == 0:
        div //= 2
        fact = 0

    return ((0x100 - div) << 8) | (fact << 2) | ps


class Ch340:
    """最小可用的 CH340 端口对象。

    只需要 `write()` / `read_any()` / `close()` 三个方法，
    就能直接喂给 host/link.py 的 SerialLink。
    """

    def __init__(self, baud: int = 115200, dev=None, timeout_ms: int = DEFAULT_TIMEOUT_MS) -> None:
        import usb.core
        import usb.util

        self._usb = usb
        self._util = usb.util
        self.timeout_ms = timeout_ms
        self.dev = dev if dev is not None else usb.core.find(idVendor=VID, idProduct=PID)
        if self.dev is None:
            raise RuntimeError(
                f"没找到 CH340 ({VID:04x}:{PID:04x})。检查：\n"
                "  1. OTG 线是不是插在数据口上\n"
                "  2. 是否以 root 运行（usbfs 权限）\n"
                "  3. lsusb 能不能看到 1a86:7523"
            )

        self.dev.set_configuration()
        cfg = self.dev.get_active_configuration()
        self.intf = cfg[(0, 0)]

        self.ep_out = self._util.find_descriptor(
            self.intf,
            custom_match=lambda e: self._util.endpoint_direction(e.bEndpointAddress)
            == self._util.ENDPOINT_OUT,
        )
        self.ep_in = self._util.find_descriptor(
            self.intf,
            custom_match=lambda e: self._util.endpoint_direction(e.bEndpointAddress)
            == self._util.ENDPOINT_IN,
        )
        if self.ep_out is None or self.ep_in is None:
            raise RuntimeError("找不到 bulk 端点，设备可能不是标准 CH340")

        self._clear_halt()
        self.version = self._configure(baud)
        self._rx_buf = bytearray()

    # -- 底层 ---------------------------------------------------------
    def _clear_halt(self) -> None:
        for ep in (self.ep_out, self.ep_in):
            try:
                self.dev.clear_halt(ep)
            except Exception:  # noqa: BLE001
                pass

    def _ctrl_out(self, request: int, value: int, index: int = 0) -> None:
        self.dev.ctrl_transfer(0x40, request, value, index, None, timeout=self.timeout_ms)

    def _ctrl_in(self, request: int, length: int) -> bytes:
        return bytes(
            self.dev.ctrl_transfer(0xC0, request, 0, 0, length, timeout=self.timeout_ms)
        )

    def _configure(self, baud: int) -> int:
        """内核 ch341_configure() 的等价实现。返回芯片版本号。"""
        ver = self._ctrl_in(REQ_READ_VERSION, 2)
        version = ver[0] if ver else 0x00

        self._ctrl_out(REQ_SERIAL_INIT, 0, 0)

        val = get_divisor(baud)
        # version > 0x27 的芯片必须置 bit7，否则收满 32 字节才上交
        if version > 0x27:
            val |= 1 << 7
        self._ctrl_out(REQ_WRITE_REG, (REG_DIVISOR << 8) | REG_PRESCALER, val)

        # 0x30 及以上版本才有独立的 LCR 寄存器
        if version >= 0x30:
            self._ctrl_out(REQ_WRITE_REG, (REG_LCR2 << 8) | REG_LCR, LCR_8N1)

        # DTR + RTS 拉高
        self._ctrl_out(REQ_MODEM_CTRL, (~(BIT_RTS | BIT_DTR)) & 0xFFFF, 0)
        return version

    # -- 数据通路 -----------------------------------------------------
    def write(self, data: bytes) -> int:
        if not data:
            return 0
        return int(self.ep_out.write(data, timeout=self.timeout_ms))

    def read_any(self, max_len: int = 256) -> bytes:
        """读当前能读到的字节；没有就返回空（不阻塞主循环）。"""
        try:
            return bytes(self.ep_in.read(max_len, timeout=20))
        except self._usb.core.USBTimeoutError:
            return b""
        except Exception:  # noqa: BLE001
            return b""

    def close(self) -> None:
        try:
            self._util.dispose_resources(self.dev)
        except Exception:  # noqa: BLE001
            pass


# ==========================================================================
#  CLI
# ==========================================================================


def cmd_selftest(baud: int) -> int:
    print("打开 CH340 …")
    port = Ch340(baud)
    print(f"  ✓ 已打开，芯片版本 0x{port.version:02x}")
    print(f"  分频值 0x{get_divisor(baud):04x}  @ {baud} bps")
    print(f"  bulk OUT=0x{port.ep_out.bEndpointAddress:02x} "
          f"IN=0x{port.ep_in.bEndpointAddress:02x}")
    print("\n请把 CH340 的 TX 和 RX 短接（回环测试）")
    input("  接好后按回车… ")

    payload = bytes(range(0x20, 0x30))
    n = port.write(payload)
    print(f"  写入 {n} 字节: {payload.hex(' ')}")

    got = b""
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.5 and len(got) < len(payload):
        chunk = port.read_any(64)
        if chunk:
            got += chunk
        else:
            time.sleep(0.01)

    print(f"  读回 {len(got)} 字节: {got.hex(' ')}")
    ok = got.startswith(payload)
    print(f"\n{'✓ 回环通过，用户态串口可用' if ok else '✗ 回环失败'}")
    port.close()
    return 0 if ok else 1


def cmd_monitor(baud: int) -> int:
    port = Ch340(baud)
    print(f"已打开 @ {baud}，Ctrl-C 退出（十六进制输出）")
    try:
        while True:
            chunk = port.read_any(256)
            if chunk:
                print(chunk.hex(" "), flush=True)
            else:
                time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        port.close()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true", help="TX-RX 短接做回环")
    ap.add_argument("--monitor", action="store_true", help="十六进制打印串口输入")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    if args.selftest:
        return cmd_selftest(args.baud)
    if args.monitor:
        return cmd_monitor(args.baud)
    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
