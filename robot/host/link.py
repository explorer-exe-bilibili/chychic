"""串口链路：收发、看护、以及跑不了硬件时的仿真链路。"""

from __future__ import annotations

import math
import time
from typing import Protocol

import protocol as p


class Link(Protocol):
    def send_vel(self, v: float, w: float) -> None: ...
    def send_mode(self, mode: int) -> None: ...
    def send_ping(self) -> None: ...
    def send_raw(self, builder) -> None: ...
    def estop(self) -> None: ...
    def poll(self) -> list[p.State]: ...
    def close(self) -> None: ...
    @property
    def stats(self) -> dict: ...


class SerialLink:
    """真实串口链路。

    读是非阻塞的：poll() 把已到达的字节全部消化，返回新状态。
    发送端要按固定频率调 send_vel，下位机有 200ms 看门狗。
    """

    def __init__(self, port, baud: int = 115200) -> None:
        # port 可以是串口路径，也可以是已经打开的对象（例如 Android 的 Ch340）
        if hasattr(port, "write") and hasattr(port, "read_any"):
            self._ser = port
        else:
            # serial_for_url 同时支持 /dev/ttyUSB0、socket://host:port 和 rfc2217://
            import serial  # 延迟导入，--sim 模式下不需要 pyserial

            self._ser = serial.serial_for_url(port, baudrate=baud, timeout=0)
        self._dec = p.Decoder()
        self._seq = 0
        self._sent = 0
        self.lift: p.LiftState | None = None

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def send_vel(self, v: float, w: float) -> None:
        """上位机侧单位是 m/s、rad/s；线路上按 MCU 的 mm/s 发。"""
        self._ser.write(p.encode_vel_mm(v * 1000.0, w, self._next_seq()))
        self._sent += 1

    def send_mode(self, mode: int) -> None:
        self._ser.write(p.encode_mode(mode, self._next_seq()))

    def send_ping(self) -> None:
        self._ser.write(p.encode_ping(self._next_seq()))

    def send_raw(self, builder) -> None:
        """发送任意帧。builder(seq) -> bytes，由 link 统一维护序号。"""
        self._ser.write(builder(self._next_seq()))

    def estop(self) -> None:
        self._ser.write(p.encode_estop(self._next_seq()))

    def poll(self) -> list[p.State]:
        # Android 的 Ch340 没有 in_waiting，用 read_any 直接取
        if hasattr(self._ser, "read_any"):
            data = self._ser.read_any(256)
        else:
            # timeout=0 时 read 是非阻塞的，返回当前可读的全部字节。
            # 不能用 in_waiting 判断：pyserial 的 socket:// 实现里它恒返回 1，
            # 会把 43 字节的一帧拆成 43 次读取。
            data = self._ser.read(4096)
        if not data:
            return []
        states: list[p.State] = []
        for cmd, payload in self._dec.feed(data):
            if cmd == p.FB_STATE:
                st = p.parse_state(payload)
                if st is not None:
                    states.append(st)
            elif cmd == p.FB_LIFT_STATE:
                lift = p.parse_lift_state(payload)
                if lift is not None:
                    self.lift = lift
        return states

    def close(self) -> None:
        self._ser.close()

    @property
    def stats(self) -> dict:
        return {"frames": self._dec.frames, "crc_errors": self._dec.crc_errors, "sent": self._sent}


class SimLink:
    """仿真链路：不需要任何硬件就能把整条算法链路跑通。

    用一个简化的独轮+红外模型：给定 (v, w) 就按赛道几何推进，
    红外位根据横向偏移生成。目的是验证状态机/日志/视觉接口，不是验证控制效果。
    """

    def __init__(self, seed_ir: str = "00100", ticks_per_meter: float = 5253.0) -> None:
        self.t = 0.0
        self.v = 0.0
        self.w = 0.0
        self.mode = p.MODE_IDLE
        self.x = 0.0  # 横向偏移（m），相对赛道中心线
        self.y = 0.0  # 沿赛道里程（m）
        self.heading = 0.0  # 航向（rad），用于仿真里的赛道推进
        self.enc_l = 0
        self.enc_r = 0
        self.tpm = ticks_per_meter
        self._last = time.monotonic()
        self._status = p.ST_ENCODER
        self.lift: p.LiftState | None = None

    def send_vel(self, v: float, w: float) -> None:
        self.v, self.w = v, w

    def send_mode(self, mode: int) -> None:
        self.mode = mode

    def send_ping(self) -> None:
        pass

    def send_raw(self, builder) -> None:
        pass

    def estop(self) -> None:
        self.mode = p.MODE_ESTOP
        self._status |= p.ST_ESTOP

    def _ir_word(self) -> int:
        """按横向偏移合成五路红外。线宽 18mm，传感器间距按 15mm 近似。"""
        pitch = 0.015
        word = 0
        for i in range(5):
            pos = (i - 2) * pitch
            if abs(self.x - pos) < 0.012:
                word |= 1 << i
        return word or (0b00100 if abs(self.x) < 0.02 else 0)

    def poll(self) -> list[p.State]:
        now = time.monotonic()
        dt = now - self._last
        self._last = now
        # 循迹和手动都算"在跑"；只有 IDLE / ESTOP 才强制零速
        if self.mode not in (p.MODE_LINE_FOLLOW, p.MODE_MANUAL):
            self.v = self.w = 0.0

        # 差速运动学
        b = 0.252
        v_l = self.v - self.w * b * 0.5
        v_r = self.v + self.w * b * 0.5

        self.heading += self.w * dt
        self.x += math.sin(self.heading) * self.v * dt
        self.y += math.cos(self.heading) * self.v * dt

        self.enc_l += int(v_l * self.tpm * dt)
        self.enc_r += int(v_r * self.tpm * dt)
        self.t += dt

        mask = self._ir_word()
        # 仿真里也走一遍真实 MCU 的状态判定，方便上位机调试状态机
        if self.mode != p.MODE_LINE_FOLLOW:
            lf_state = p.LF_DISABLED
        elif mask == 0:
            lf_state = p.LF_LOST
        else:
            lf_state = p.LF_TRACKING

        return [
            p.State(
                tick_ms=int(self.t * 1000),
                mode=self.mode,
                lf_state=lf_state,
                line_mask=mask,
                lf_error=self.x * 100.0,  # 仿真里直接把横向偏移当偏差
                v_cmd=self.v * 1000.0,
                w_cmd=self.w,
                v_l=v_l * 1000.0,
                v_r=v_r * 1000.0,
                enc_l=self.enc_l,
                enc_r=self.enc_r,
                status=self._status,
            )
        ]

    def close(self) -> None:
        pass

    @property
    def stats(self) -> dict:
        return {"frames": 0, "crc_errors": 0, "sent": 0}


def open_link(port: str | None, baud: int = 115200) -> Link:
    if port in (None, "", "sim"):
        return SimLink()
    if port in ("android", "ch340"):
        # 手机当主控：内核没有 USB 串口驱动，走用户态 CH340
        from android_ch340 import Ch340

        return SerialLink(Ch340(baud), baud)
    return SerialLink(port, baud)
