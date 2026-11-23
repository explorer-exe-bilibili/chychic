#!/usr/bin/env python3
"""主循环：相机 -> 视觉 -> 状态机 -> 串口 -> STM32，20ms 一拍。

用法

    # 不接硬件，把整条链路跑通（先跑这个确认代码没坏）
    python3 main.py --sim

    # 接车
    python3 main.py --port /dev/ttyUSB0

    # 只跑红外，不启用视觉（怀疑相机有问题时）
    python3 main.py --port /dev/ttyUSB0 --no-vision

键盘
    空格  急停
    q     退出
"""

from __future__ import annotations

import argparse
import math
import select
import sys
import time

try:  # 只在 Linux 上有；Windows 下不接键盘，方便在笔记本上先跑 --sim
    import termios
    import tty
except ImportError:  # pragma: no cover
    termios = None  # type: ignore[assignment]
    tty = None  # type: ignore[assignment]

import link as link_mod
import params
import protocol as p
from fsm import S, Mission
from line_follow import LineConfig, LineFollower
from logger import RunLogger
from odometry import Odometry

LOOP_DT = 0.020  # 50 Hz


def _parse_source(text: str) -> int | str:
    """'0' -> 0（USB 相机）；'http://...' 或 '/dev/video0' 原样传下去。"""
    text = str(text).strip()
    return int(text) if text.isdigit() else text


def _check_key() -> str | None:
    """非阻塞读一个键，没有就返回 None。"""
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if not dr:
        return None
    return sys.stdin.read(1)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--port",
        default="sim",
        help="串口设备，如 /dev/ttyUSB0；android 表示手机用户态 CH340；sim 表示仿真",
    )
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument(
        "--camera",
        default="0",
        help="相机源：0 为 USB 相机；也可给 URL，如 http://127.0.0.1:8080/video（手机 MJPEG）",
    )
    ap.add_argument("--sim", action="store_true", help="强制仿真链路")
    ap.add_argument("--no-vision", action="store_true")
    ap.add_argument("--no-qr", action="store_true", help="跳过二维码，直接开跑")
    ap.add_argument("--video", action="store_true", help="录制 mp4")
    ap.add_argument("--duration", type=float, default=0.0, help="跑多少秒后自动停，0=不限")
    ap.add_argument("--logdir", default="logs")
    args = ap.parse_args()

    params.load_override()

    port = "sim" if args.sim else args.port
    link = link_mod.open_link(port, args.baud)
    cam = None
    detector = None
    qr = None
    parse_task = None
    if not args.no_vision:
        try:
            from vision import Camera, LineDetector, QRReader, parse_task as _parse_task

            parse_task = _parse_task
            cam = Camera(_parse_source(args.camera))
            detector = LineDetector()
            if not args.no_qr:
                qr = QRReader()
        except Exception as exc:  # noqa: BLE001
            print(f"[warn] 视觉不可用，退回纯红外：{exc}", file=sys.stderr)
            cam, detector, qr = None, None, None

    follower = LineFollower(LineConfig(v_cruise=params.V_CRUISE))
    odo = Odometry(params.WHEEL_BASE, params.TICKS_PER_METER)
    log = RunLogger(args.logdir, name="run", video=args.video)
    mission = Mission(follower, log)
    mission.goto(S.WAIT_QR if not args.no_qr else S.SEG_A)

    last_state: p.State | None = None
    last_frame_seq = -1
    vision_stale = 0.0
    mode_sent = -1
    t_start = time.monotonic()
    t_last = t_start

    # 终端切到非规范模式，才能不按回车读单键
    old_attr = None
    if termios is not None and sys.stdin.isatty():
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    try:
        while True:
            now = time.monotonic()
            dt = now - t_last
            t_last = now
            if dt <= 0:
                dt = LOOP_DT

            # --- 读下位机 -------------------------------------------------
            states = link.poll()
            if states:
                last_state = states[-1]
            ir = last_state.ir if last_state else 0
            pose = odo.update(last_state.enc_l, last_state.enc_r) if last_state else odo.pose
            if last_state and last_state.watchdog_tripped:
                log.event("LINK", "下位机看门狗超时，说明指令没发出去")

            # --- 读相机 ---------------------------------------------------
            det = None
            frame = None
            if cam is not None:
                frame, seq = cam.read()
                if seq == last_frame_seq:
                    vision_stale += dt
                else:
                    vision_stale = 0.0
                last_frame_seq = seq
                if frame is not None and vision_stale < 0.6:
                    det = detector.detect(frame)
                elif vision_stale >= 0.6:
                    if int(vision_stale * 50) % 50 == 0:
                        log.event("VISION", f"帧卡住 {vision_stale:.1f}s，退回红外")

            # --- 扫二维码 -------------------------------------------------
            if mission.state is S.WAIT_QR and qr is not None and frame is not None:
                text = qr.read(frame)
                if text:
                    task = parse_task(text)
                    if task:
                        mission.on_qr(task)
                    else:
                        log.event("QR", f"扫到但解析失败: {text!r}")

            # --- 状态机 ---------------------------------------------------
            offset = det.offset if det else None
            conf = det.confidence if det else 0.0
            out = mission.step(ir, offset, conf, dt)

            # --- 下发 -----------------------------------------------------
            want_mode = p.MODE_RUN if mission.state not in (S.IDLE, S.DONE, S.FAILSAFE) else p.MODE_IDLE
            if want_mode != mode_sent:
                link.send_mode(want_mode)
                mode_sent = want_mode
                log.event("LINK", f"MODE -> {want_mode}")
            if want_mode == p.MODE_RUN:
                link.send_vel(out.v, out.w)
            else:
                link.send_vel(0.0, 0.0)

            # --- 记录 -----------------------------------------------------
            log.row(
                t=round(now - t_start, 3),
                state=mission.state.name,
                ir=ir,
                err=round(out.error, 4),
                src=out.source,
                v_cmd=round(out.v, 4),
                w_cmd=round(out.w, 4),
                v_fb=round(last_state.v, 4) if last_state else 0.0,
                vision=round(offset, 4) if offset is not None else 9.9,
                conf=round(conf, 3),
                enc_l=last_state.enc_l if last_state else 0,
                enc_r=last_state.enc_r if last_state else 0,
                x_mm=round(pose.x * 1000, 1),
                y_mm=round(pose.y * 1000, 1),
                th_deg=round(math.degrees(pose.theta), 2),
            )
            if args.video and frame is not None:
                log.frame(frame)

            # --- 键盘 -----------------------------------------------------
            key = _check_key() if old_attr is not None else None
            if key == " ":
                log.event("KEY", "手动急停")
                link.estop()
                mission.fail("manual estop")
            elif key in ("q", "Q"):
                log.event("KEY", "退出")
                break

            # --- 收工判断 -------------------------------------------------
            if mission.state in (S.DONE, S.FAILSAFE):
                log.event("RUN", f"结束于 {mission.state.name}")
                break
            if args.duration and (now - t_start) > args.duration:
                log.event("RUN", f"到达 {args.duration}s")
                break

            # --- 定频 -----------------------------------------------------
            sleep = LOOP_DT - (time.monotonic() - now)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        log.event("RUN", "Ctrl-C")
    finally:
        try:
            link.estop()
            link.send_mode(p.MODE_IDLE)
        except Exception:  # noqa: BLE001
            pass
        if old_attr is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
        if cam is not None:
            cam.close()
        log.event("LINK", f"stats={link.stats}")
        log.close()
        link.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
