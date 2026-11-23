"""上下位机通信协议 —— 与 firmware/App/Src/protocol.c 严格对应。

帧格式（大端）

    0xAA 0x55 | SEQ | CMD | LEN | PAYLOAD(LEN) | CRC_H | CRC_L

CRC16 = CCITT-FALSE (poly=0x1021, init=0xFFFF)，覆盖 SEQ..PAYLOAD。
详见 docs/protocol.md
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

SOF1 = 0xAA
SOF2 = 0x55
MAX_PAYLOAD = 64

# --- 上位机 -> 下位机 -------------------------------------------------------
CMD_VEL = 0x10
CMD_MODE = 0x11
CMD_ESTOP = 0x12
CMD_PING = 0x13

# --- 下位机 -> 上位机 -------------------------------------------------------
FB_STATE = 0x80
FB_ACK = 0x81

# --- mode ------------------------------------------------------------------
# 语义对应 MCU 的 app_line_follow / chassis 接口：
#   IDLE        电机断使能，可手推
#   LINE_FOLLOW MCU 自主循迹（上位机不下发速度，只切模式）
#   MANUAL      听 CMD_VEL，用于 A 区绕行、C 区对中这类"脱离黑线"的动作
#   ESTOP       立即刹车并锁死，必须显式切回 IDLE 才解除
MODE_IDLE = 0
MODE_LINE_FOLLOW = 1
MODE_MANUAL = 2
MODE_ESTOP = 3

#: 兼容旧名
MODE_RUN = MODE_LINE_FOLLOW
MODE_CALIB = MODE_IDLE

# status 位
ST_ENCODER = 1 << 0
ST_LOW_VOLT = 1 << 1
ST_WATCHDOG = 1 << 2
ST_ESTOP = 1 << 3
ST_STALL = 1 << 4

#: FB_STATE 的 payload 布局，必须和 MCU 侧打包函数逐字节一致（大端）：
#:   tick_ms(u32) mode(u8) lf_state(u8) line_mask(u8)
#:   lf_error(f32) v_cmd(f32) w_cmd(f32) v_l(f32) v_r(f32)
#:   enc_l(i32) enc_r(i32) status(u8)
_STATE_FMT = ">IBBBfffffiiB"
_STATE_SIZE = struct.calcsize(_STATE_FMT)
assert _STATE_SIZE == 36, _STATE_SIZE

#: 循迹状态，对应 MCU 的 line_follow_state_t
LF_DISABLED = 0
LF_ACQUIRING = 1
LF_TRACKING = 2
LF_LOST = 3
LF_WIDE_LINE = 4
LF_AMBIGUOUS = 5

LF_STATE_CN = {
    LF_DISABLED: "未启用",
    LF_ACQUIRING: "找线中",
    LF_TRACKING: "循迹中",
    LF_LOST: "丢线",
    LF_WIDE_LINE: "宽线",
    LF_AMBIGUOUS: "多线段",
}


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE，和 STM32 侧的位运算实现逐位一致。"""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode(cmd: int, payload: bytes = b"", seq: int = 0) -> bytes:
    """组一帧。seq 由调用方维护，0-255 回绕。"""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)}")
    body = bytes((seq & 0xFF, cmd & 0xFF, len(payload))) + payload
    crc = crc16_ccitt(body)
    return bytes((SOF1, SOF2)) + body + bytes((crc >> 8, crc & 0xFF))


def encode_vel(v: float, w: float, seq: int = 0) -> bytes:
    return encode(CMD_VEL, struct.pack(">ff", v, w), seq)


def encode_mode(mode: int, seq: int = 0) -> bytes:
    return encode(CMD_MODE, struct.pack(">B", mode), seq)


def encode_estop(seq: int = 0) -> bytes:
    return encode(CMD_ESTOP, b"", seq)


def encode_ping(seq: int = 0) -> bytes:
    return encode(CMD_PING, b"", seq)


def encode_vel_mm(v_mm_s: float, w_rad_s: float, seq: int = 0) -> bytes:
    """车速指令。单位 mm/s + rad/s，和 MCU 的 chassis_set_velocity 一致。"""
    return encode(CMD_VEL, struct.pack(">ff", v_mm_s, w_rad_s), seq)


@dataclass(frozen=True)
class State:
    """FB_STATE 解码结果。

    速度单位是 **mm/s**（和 MCU 的 chassis 接口一致），不是上位机内部的 m/s。
    跨层时用 ``v_m_s`` 换算，别把 ``v_l`` 直接塞进控制器。
    """

    tick_ms: int
    mode: int
    lf_state: int
    line_mask: int
    lf_error: float
    v_cmd: float  # 车体线速度目标 mm/s
    w_cmd: float  # 车体角速度目标 rad/s
    v_l: float  # 左轮实际 mm/s
    v_r: float  # 右轮实际 mm/s
    enc_l: int
    enc_r: int
    status: int

    # -- 兼容旧调用 ---------------------------------------------------
    @property
    def ir(self) -> int:
        """五路红外归一化位图（bit0 最左，1 = 黑线）。"""
        return self.line_mask

    @property
    def v_m_s(self) -> float:
        """车体线速度，m/s。"""
        return (self.v_l + self.v_r) * 0.5 / 1000.0

    @property
    def v(self) -> float:
        """``v_m_s`` 的别名，给旧日志代码用。"""
        return self.v_m_s

    def w_rad_s(self, wheel_base_m: float) -> float:
        """由左右轮速反算角速度，rad/s。"""
        if wheel_base_m <= 0:
            return 0.0
        return (self.v_r - self.v_l) / 1000.0 / wheel_base_m

    @property
    def ir_bits(self) -> list[bool]:
        """五路红外，从左到右。"""
        return [bool(self.line_mask & (1 << i)) for i in range(5)]

    @property
    def lf_state_name(self) -> str:
        return LF_STATE_CN.get(self.lf_state, f"未知({self.lf_state})")

    @property
    def watchdog_tripped(self) -> bool:
        return bool(self.status & ST_WATCHDOG)

    @property
    def estop(self) -> bool:
        return bool(self.status & ST_ESTOP)

    def describe(self) -> str:
        bits = "".join("1" if b else "0" for b in self.ir_bits)
        return (
            f"t={self.tick_ms:>7} mode={self.mode} {self.lf_state_name:<4} "
            f"线={bits} err={self.lf_error:+.2f} "
            f"轮=({self.v_l:+.0f},{self.v_r:+.0f})mm/s st=0x{self.status:02x}"
        )


class Decoder:
    """流式解析器：喂任意切分的字节，吐出完整的 (cmd, payload)。

    解析器对垃圾数据免疫：任何时候失步都会重新找 0xAA 0x55。
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.crc_errors = 0
        self.frames = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self._buf.extend(data)
        out: list[tuple[int, bytes]] = []

        while True:
            start = self._buf.find(b"\xAA\x55")
            if start < 0:
                # 没有帧头，只保留可能是半个帧头的一个字节
                if len(self._buf) > 1:
                    del self._buf[: len(self._buf) - 1]
                break
            if start:
                del self._buf[:start]

            if len(self._buf) < 5:
                break  # 头部都不齐

            length = self._buf[4]
            if length > MAX_PAYLOAD:
                del self._buf[:2]  # 假帧头，跳过继续找
                continue

            total = 7 + length  # 2 SOF + 3 头 + payload + 2 CRC
            if len(self._buf) < total:
                break  # 等后续字节

            body = bytes(self._buf[2 : 5 + length])
            crc_rx = (self._buf[5 + length] << 8) | self._buf[6 + length]
            if crc16_ccitt(body) != crc_rx:
                self.crc_errors += 1
                del self._buf[:2]
                continue

            seq, cmd = body[0], body[1]
            del seq  # 目前不用于重排，保留给丢帧统计
            out.append((cmd, body[3:]))
            self.frames += 1
            del self._buf[:total]

        return out


def parse_state(payload: bytes) -> State | None:
    if len(payload) != _STATE_SIZE:
        return None
    return State(*struct.unpack(_STATE_FMT, payload))
