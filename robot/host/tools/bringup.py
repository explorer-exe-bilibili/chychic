#!/usr/bin/env python3
"""上车标定：一条命令走完，输出能直接抄进 params.py 的数值。

    python3 tools/bringup.py --port /dev/ttyUSB0

四步，每步都会等你确认，随时 Ctrl-C 都会刹车。

  1. 连通性    —— 能不能收到下位机的 FB_STATE
  2. 编码器方向 —— 两个轮子推着往前走，计数要变大
  3. 一米标定  —— ticks_per_meter
  4. 原地转一圈 —— 轮距 wheel_base

**第一次跑的时候把车架起来**，轮子离地，只做 1、2 步验证方向；
3、4 步必须落地做，不然测的是空转。
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import link as link_mod
import params
import protocol as p


class Commander(threading.Thread):
    """后台 50Hz 持续下发速度指令（否则下位机 200ms 看门狗会刹车）。"""

    def __init__(self, link, v: float, w: float) -> None:
        super().__init__(daemon=True)
        self.link = link
        self.v = v
        self.w = w
        self._alive = True

    def run(self) -> None:
        while self._alive:
            self.link.send_vel(self.v, self.w)
            time.sleep(0.02)

    def stop(self) -> None:
        self._alive = False
        self.join(timeout=1.0)


def latest(link, timeout: float = 2.0):
    """阻塞等到一帧状态，超时返回 None。"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        states = link.poll()
        if states:
            return states[-1]
        time.sleep(0.01)
    return None


def drain(link) -> None:
    for _ in range(20):
        link.poll()
        time.sleep(0.005)


def ask(msg: str) -> None:
    input(f"\n>>> {msg}\n    准备好后按回车… ")


def step_connect(link) -> bool:
    print("\n[1/4] 连通性")
    link.send_mode(p.MODE_IDLE)
    st = latest(link, 2.0)
    if st is None:
        print("  ✗ 2 秒内没收到 FB_STATE。检查：串口号、波特率 115200、TX/RX 是否交叉、共地。")
        return False
    print(f"  ✓ 收到状态：{st.describe()}")
    if not (st.status & p.ST_ENCODER):
        print("  ⚠ status 的编码器位没置起来，确认下位机已经使能编码器")
    return True


def _push_test(link, wheel: str) -> int | None:
    link.send_mode(p.MODE_CALIB)
    drain(link)
    st0 = latest(link)
    if st0 is None:
        return None
    ask(f"用手把【{wheel}轮】向车头方向推约 20cm")
    st1 = latest(link)
    if st1 is None:
        return None
    base = st0.enc_l if wheel == "左" else st0.enc_r
    now = st1.enc_l if wheel == "左" else st1.enc_r
    return now - base


def step_encoder(link) -> tuple[int, int]:
    print("\n[2/4] 编码器方向")
    d_l = _push_test(link, "左") or 0
    d_r = _push_test(link, "右") or 0
    print(f"  左轮 Δ={d_l:+d}   右轮 Δ={d_r:+d}")
    if d_l < 0:
        print("  ⚠ 左轮方向反了：把 params 里的 sign_l 改成 -1，或者对调编码器 A/B 相")
    if d_r < 0:
        print("  ⚠ 右轮方向反了：把 params 里的 sign_r 改成 -1，或者对调编码器 A/B 相")
    if d_l > 0 and d_r > 0:
        print("  ✓ 方向都对")
    return d_l, d_r


def step_ticks(link, tpm_hint: float) -> float:
    print("\n[3/4] 一米标定 (TICKS_PER_METER)")
    print(f"  当前值 {tpm_hint:.0f}，按公式估算约 {tpm_hint:.0f}")
    link.send_mode(p.MODE_CALIB)
    drain(link)
    st0 = latest(link)
    if st0 is None:
        return tpm_hint
    ask("在地上量出正好 1.000 m，把车推过去（沿直线，别歪）")
    st1 = latest(link)
    if st1 is None:
        return tpm_hint
    d_l = abs(st1.enc_l - st0.enc_l)
    d_r = abs(st1.enc_r - st0.enc_r)
    ticks = 0.5 * (d_l + d_r)
    print(f"  左 {d_l}  右 {d_r}  平均 {ticks:.1f}")
    if ticks < 100:
        print("  ✗ 数值太小，是不是没推动或者编码器没接")
        return tpm_hint
    print(f"  ✓ TICKS_PER_METER = {ticks:.0f}")
    return ticks


def step_wheelbase(link, tpm: float, b_hint: float) -> float:
    print("\n[4/4] 轮距标定 (WHEEL_BASE)")
    print(f"  当前值 {b_hint:.3f} m。这一步会让车原地转圈，**确认场地够大、车不会被线绊住**")
    link.send_mode(p.MODE_CALIB)
    drain(link)
    st0 = latest(link)
    if st0 is None:
        return b_hint

    link.send_mode(p.MODE_RUN)
    time.sleep(0.1)
    cmd = Commander(link, 0.0, 0.6)  # 逆时针慢转
    cmd.start()
    ask("车开始原地转圈。盯着它，**转满一整圈（360°）时立刻按回车**")
    cmd.stop()
    link.send_vel(0.0, 0.0)
    link.send_mode(p.MODE_IDLE)

    st1 = latest(link)
    if st1 is None:
        return b_hint
    d_l = abs(st1.enc_l - st0.enc_l) / tpm
    d_r = abs(st1.enc_r - st0.enc_r) / tpm
    # 原地转一圈：每个轮子走的弧长 = pi * b
    b = (d_l + d_r) / 2.0 / math.pi
    print(f"  左弧长 {d_l:.3f} m  右弧长 {d_r:.3f} m")
    if b < 0.05 or b > 1.0:
        print(f"  ✗ 算出来 {b:.3f} m 不合理，转圈没转准或者编码器有问题，保留原值")
        return b_hint
    print(f"  ✓ WHEEL_BASE = {b:.3f} m")
    return b


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    link = link_mod.open_link(args.port, args.baud)
    print(f"已打开 {args.port} @ {args.baud}")

    try:
        if not step_connect(link):
            return 1
        step_encoder(link)
        tpm = step_ticks(link, params.TICKS_PER_METER)
        b = step_wheelbase(link, tpm, params.WHEEL_BASE)
    except KeyboardInterrupt:
        print("\n中断")
        return 130
    finally:
        try:
            link.estop()
            link.send_mode(p.MODE_IDLE)
        except Exception:  # noqa: BLE001
            pass
        link.close()

    print("\n" + "=" * 58)
    print("把下面几行抄进 host/params.py（固件里的 APP_* 也要同步）：")
    print("=" * 58)
    print(f"WHEEL_BASE      = {b:.3f}")
    print(f"TICKS_PER_METER = {tpm:.0f}")
    print(f"// firmware/App/Src/app.c")
    print(f"#define APP_WHEEL_BASE      {b:.3f}f")
    print(f"#define APP_TICKS_PER_METER {tpm:.0f}f")
    print("=" * 58)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
