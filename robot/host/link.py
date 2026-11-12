"""串口链路：收发、看护、以及跑不了硬件时的仿真链路。"""

from __future__ import annotations

import math
import time
from typing import Protocol

import protocol as p


class Link(Protocol):
    def send_vel(self, v: float, w: float) -> None: ...
    def send_mode(self, mode: int) -> None: ...
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

    def __init__(self, port: str, baud: int = 115200) -> None:
        import serial  # 延迟导入，--sim 模式下不需要 pyserial

        self._ser = serial.Serial(port, baud, timeout=0)
        self._dec = p.Decoder()
        self._seq = 0
        self._sent = 0

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def send_vel(self, v: float, w: float) -> None:
        self._ser.write(p.encode_vel(v, w, self._next_seq()))
        self._sent += 1

    def send_mode(self, mode: int) -> None:
        self._ser.write(p.encode_mode(mode, self._next_seq()))

    def estop(self) -> None:
        self._ser.write(p.encode_estop(self._next_seq()))

    def poll(self) -> list[p.State]:
        waiting = self._ser.in_waiting
        if not waiting:
            return []
        data = self._ser.read(waiting)
        states: list[p.State] = []
        for cmd, payload in self._dec.feed(data):
            if cmd == p.FB_STATE:
                st = p.parse_state(payload)
                if st is not None:
                    states.append(st)
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

    def __init__(self, seed_ir: str = "00100") -> None:
        self.t = 0.0
        self.v = 0.0
        self.w = 0.0
        self.mode = p.MODE_IDLE
        self.x = 0.0  # 横向偏移（m），相对赛道中心线
        self.y = 0.0  # 沿赛道里程（m）
        self.enc_l = 0
        self.enc_r = 0
        self._last = time.monotonic()
        self._status = p.ST_ENCODER

    def send_vel(self, v: float, w: float) -> None:
        self.v, self.w = v, w

    def send_mode(self, mode: int) -> None:
        self.mode = mode

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
        if self.mode != p.MODE_RUN:
            self.v = self.w = 0.0

        # 位移更新
        self.y += self.v * dt
        self.x += math.sin(self.w) * self.v * dt * 0.5

        self.enc_l += int(self.v * 1000 * dt)
        self.enc_r += int(self.v * 1000 * dt)
        self.t += dt

        return [
            p.State(
                tick_ms=int(self.t * 1000),
                v_l=self.v,
                v_r=self.v,
                enc_l=self.enc_l,
                enc_r=self.enc_r,
                ir=self._ir_word(),
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
    return SerialLink(port, baud)
