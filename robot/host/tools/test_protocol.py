#!/usr/bin/env python3
"""协议自测 —— 不接硬件先跑这个。

    python3 tools/test_protocol.py

它做两件事：
1. 验 CRC 和组帧/解帧的自洽性（分片喂、夹杂垃圾字节、故意坏 CRC）
2. 打印一条 FB_STATE 的逐字节十六进制，方便和 STM32 抓到的串口数据对拍
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import protocol as p


def check(name: str, cond: bool) -> bool:
    print(f"{'PASS' if cond else 'FAIL'}  {name}")
    return cond


def main() -> int:
    ok = True

    # 1. CRC 标准向量：CRC-16/CCITT-FALSE("123456789") = 0x29B1
    ok &= check("CRC16 CCITT-FALSE 标准向量", p.crc16_ccitt(b"123456789") == 0x29B1)

    # 2. 速度指令往返
    frame = p.encode_vel(0.35, -0.42, seq=7)
    dec = p.Decoder()
    out = dec.feed(frame)
    ok &= check("CMD_VEL 往返", len(out) == 1 and out[0][0] == p.CMD_VEL)
    v_rx, w_rx = struct.unpack(">ff", out[0][1])
    ok &= check(
        "CMD_VEL 数值正确（float32 精度内）",
        abs(v_rx - 0.35) < 1e-5 and abs(w_rx + 0.42) < 1e-5,
    )

    # 3. 分片喂（模拟串口一次只来几个字节）
    dec = p.Decoder()
    got = []
    for i in range(0, len(frame), 3):
        got += dec.feed(frame[i : i + 3])
    ok &= check("分片输入能还原", len(got) == 1)

    # 4. 前面塞垃圾字节，要能重新同步
    dec = p.Decoder()
    got = dec.feed(b"\x00\x11\x22\xAA\x01\xAA" + frame)
    ok &= check("垃圾前缀能重新同步", len(got) == 1)

    # 5. 坏 CRC 必须丢帧
    bad = bytearray(frame)
    bad[-1] ^= 0xFF
    dec = p.Decoder()
    got = dec.feed(bytes(bad))
    ok &= check("坏 CRC 被丢弃", len(got) == 0 and dec.crc_errors == 1)

    # 6. FB_STATE 解码
    payload = struct.pack(
        ">IBBBfffffiiB",
        12345,
        p.MODE_LINE_FOLLOW,
        p.LF_TRACKING,
        0b00100,
        0.35,
        300.0,
        0.0,
        298.0,
        302.0,
        -120,
        88,
        p.ST_ENCODER,
    )
    st = p.parse_state(payload)
    ok &= check("FB_STATE 长度 36", len(payload) == 36)
    ok &= check(
        "FB_STATE 解码",
        st is not None and st.tick_ms == 12345 and st.enc_l == -120,
    )
    ok &= check("红外位解析", st is not None and st.ir_bits == [False, False, True, False, False])
    ok &= check("循迹状态文本", st is not None and st.lf_state_name == "循迹中")
    ok &= check("轮速换算 m/s", st is not None and abs(st.v_m_s - 0.300) < 1e-6)

    # 7. 给 STM32 侧对拍用的金帧
    golden = p.encode_vel(0.5, 0.0, seq=1)
    print("\nFB_STATE 金帧（把这个给下位机同学逐字节核对）")
    print("  CMD_VEL payload <float 0.5, float 0.0>:")
    print("  " + " ".join(f"{b:02X}" for b in golden))
    print(f"  长度 {len(golden)}，CRC 应等于 0x{p.crc16_ccitt(golden[2:5 + golden[4]]):04X}")

    print()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
