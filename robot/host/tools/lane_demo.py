#!/usr/bin/env python3
"""摄像头循迹 demo —— 先在电脑上把控制律跑通，再上真车。

三种模式：

    python3 tools/lane_demo.py --synthetic      # 合成画面闭环（默认，不用硬件）
    python3 tools/lane_demo.py --camera 0       # 电脑摄像头
    python3 tools/lane_demo.py --video run.mp4  # 录好的赛道视频

合成模式是一个**真正的闭环**：渲染一张"车载相机看到的赛道图" → 喂给真实的
LineDetector → 真实的 LineFollower 算出 (v, w) → 推进车辆模型 → 重新渲染。
所以它验证的是算法本身，不是画图。

按键：q 退出，r 重置，+/- 调巡航速度，s 存当前帧。

只跑数据不开窗口（远程/服务器上用）：
    python3 tools/lane_demo.py --synthetic --headless --duration 10
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from line_follow import LineConfig, LineFollower  # noqa: E402
from vision.line import LineDetector  # noqa: E402

# ---- 相机与赛道参数（合成模式用，可改）------------------------------------
IMG_W, IMG_H = 640, 480
FX = 620.0            # 焦距（像素）。越大视野越窄
CX = IMG_W / 2.0
ROI_TOP = 0.55        # 只看下 45%
D_NEAR = 0.15         # 图像最下一行对应的前方距离（米）
D_FAR = 1.20          # ROI 最上一行对应的距离（米）
LINE_WIDTH_M = 0.020  # 线宽 20mm（实车 18mm）
BG_GRAY = 190         # 背景灰度
LINE_GRAY = 25        # 线灰度


def render_track_view(lat: float, phi: float, kappa: float) -> np.ndarray:
    """渲染一张车载相机视角的赛道图。

    lat   ：车相对赛道中心的横向偏移，正 = 车在线的右侧（米）
    phi   ：车头相对赛道切线的航向误差，正 = 车头偏左（弧度）
    kappa ：前方赛道曲率，正 = 向左弯（1/m）
    """
    img = np.full((IMG_H, IMG_W, 3), BG_GRAY, np.uint8)
    y0 = int(IMG_H * ROI_TOP)

    for y in range(y0, IMG_H):
        t = (y - y0) / max(1, (IMG_H - 1 - y0))     # 0=远, 1=近
        d = D_FAR + (D_NEAR - D_FAR) * t

        # 前方 d 米处，赛道中心相对车的横向位置（右为正）
        x_rel = -lat + math.tan(phi) * d - 0.5 * kappa * d * d
        u = CX + FX * x_rel / d

        half = max(1.0, 0.5 * FX * LINE_WIDTH_M / d)
        u0 = int(round(u - half))
        u1 = int(round(u + half))
        u0c, u1c = max(0, u0), min(IMG_W, u1)
        if u1c > u0c:
            img[y, u0c:u1c] = LINE_GRAY

    # 一点传感器噪声，别让 demo 太理想化
    noise = np.random.normal(0, 2.0, (IMG_H, IMG_W, 1)).astype(np.int16)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    return img


def draw_topdown(lat: float, phi: float, path: list[tuple[float, float]], size=260) -> np.ndarray:
    """俯视图：赛道中心线 + 车。path 是 (x, y) 历史轨迹（米）。"""
    import cv2

    view = np.full((size, size, 3), 35, np.uint8)
    cx, cy = size // 2, size - 30
    scale = 90.0  # 像素/米

    # 赛道中心线（车前方 0~1.2m 是直的，用浅灰示意）
    cv2.line(view, (cx, cy), (cx, cy - int(1.2 * scale)), (110, 110, 110), 2)

    # 轨迹
    for i in range(1, len(path)):
        p0 = (int(cx + path[i - 1][0] * scale), int(cy - path[i - 1][1] * scale))
        p1 = (int(cx + path[i][0] * scale), int(cy - path[i][1] * scale))
        cv2.line(view, p0, p1, (80, 200, 80), 1)

    # 车
    car = np.array([
        [cx + int(lat * scale) - 7, cy + 12],
        [cx + int(lat * scale) + 7, cy + 12],
        [cx + int(lat * scale) + 5, cy - 10],
        [cx + int(lat * scale) - 5, cy - 10],
    ], np.int32)
    cv2.polylines(view, [car], True, (60, 160, 255), 2)
    # 车头方向
    tip = (int(cx + lat * scale - math.sin(phi) * 34), int(cy - math.cos(phi) * 34))
    cv2.line(view, (cx + int(lat * scale), cy), tip, (60, 160, 255), 2)
    return view


def hud(img: np.ndarray, lines: list[str]) -> np.ndarray:
    import cv2

    y = 22
    for s in lines:
        cv2.putText(img, s, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 3, cv2.LINE_AA)
        cv2.putText(img, s, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 1, cv2.LINE_AA)
        y += 22
    return img


class Sim:
    """车辆 + 赛道的简化闭环模型。"""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.lat = 0.06         # 初始偏右 6cm
        self.phi = 0.12         # 初始车头偏左 0.12rad
        self.s = 0.0
        self.path: list[tuple[float, float]] = [(self.lat, 0.0)]
        self.t = 0.0

    def curvature(self) -> float:
        """赛道曲率随时间变化，模拟直道 → 左弯 → 右弯。"""
        phase = self.s % 6.0
        if phase < 2.0:
            return 0.0
        if phase < 4.0:
            return 0.8
        return -0.8

    def step(self, v: float, w: float, dt: float) -> None:
        kappa = self.curvature()
        self.phi += (w - kappa * v) * dt
        self.lat += -v * math.sin(self.phi) * dt
        self.s += v * math.cos(self.phi) * dt
        self.t += dt
        if len(self.path) < 400:
            self.path.append((self.lat, 0.0))


def run_synthetic(args, follower: LineFollower, detector: LineDetector) -> int:
    import cv2

    sim = Sim()
    dt = 1.0 / args.fps
    lat_err = []
    lost = 0
    frames = 0

    t0 = time.monotonic()
    while True:
        if args.duration and sim.t >= args.duration:
            break

        frame = render_track_view(sim.lat, sim.phi, sim.curvature())
        det = detector.detect(frame)

        if det is None:
            lost += 1
            out = follower.update(0, None, dt, 0.0)
        else:
            out = follower.update(0, det.offset, dt, det.confidence)
            lat_err.append(abs(sim.lat))

        frames += 1
        if not args.headless:
            vis = frame.copy()
            # 叠加检测结果
            if det is not None:
                u = int(CX + det.offset * CX)
                cv2.line(vis, (u, int(IMG_H * ROI_TOP)), (u, IMG_H), (0, 200, 255), 2)
            hud(vis, [
                f"t={sim.t:5.1f}s  det={'OK' if det else 'LOST'}",
                f"offset={det.offset:+.2f}" if det else "offset=--",
                f"cmd v={out.v:.2f} w={out.w:+.2f}",
                f"lat={sim.lat * 1000:+.0f}mm phi={sim.phi:+.2f}rad",
                f"src={out.source}",
            ])
            cv2.imshow("lane demo", vis)
            if args.show_map:
                cv2.imshow("topdown", draw_topdown(sim.lat, sim.phi, sim.path))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                sim.reset()
                follower.reset()
                lat_err.clear()
            if key in (ord("+"), ord("=")):
                args.v_cruise = min(0.8, args.v_cruise + 0.05)
                print(f"巡航速度 -> {args.v_cruise:.2f}")
            if key == ord("-"):
                args.v_cruise = max(0.05, args.v_cruise - 0.05)
                print(f"巡航速度 -> {args.v_cruise:.2f}")

        sim.step(out.v, out.w, dt)

        # 无窗口模式下按真实时间跑，避免瞬间跑完
        if args.headless:
            time.sleep(dt)

    elapsed = time.monotonic() - t0
    print(f"\n运行 {sim.t:.1f}s，{frames} 帧，平均 {frames / max(elapsed, 1e-6):.0f} fps")
    print(f"丢线帧: {lost} / {frames}")
    if lat_err:
        arr = np.array(lat_err)
        print(f"横向误差 |lat|: 平均 {arr.mean() * 1000:.1f}mm  "
              f"最大 {arr.max() * 1000:.1f}mm  末值 {sim.lat * 1000:+.1f}mm")
        ok = arr.mean() < 0.02
        print("结论:", "收敛，控制律可用" if ok else "不收敛，需要调 Kp/Kd 或降速")
        return 0 if ok else 2
    return 1


def _parse_source(text):
    """'0' -> 0；'/dev/video0'、'http://...'、'run.mp4' 原样传给 OpenCV。"""
    text = str(text).strip()
    return int(text) if text.isdigit() else text


def run_image_source(args, follower: LineFollower, detector: LineDetector) -> int:
    """真摄像头 / 视频 / 网络流：做检测与指令计算，不闭环。"""
    import cv2

    cap = cv2.VideoCapture(_parse_source(args.camera if args.camera is not None else args.video))
    if not cap.isOpened():
        print("打不开视频源")
        return 1

    dt = 1.0 / args.fps
    frames = 0
    lost = 0
    offsets: list[float] = []
    t0 = time.monotonic()

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        det = detector.detect(frame)
        out = follower.update(0, det.offset if det else None, dt,
                              det.confidence if det else 0.0)
        frames += 1
        if det is None:
            lost += 1
        else:
            offsets.append(det.offset)

        if args.headless:
            if args.duration and time.monotonic() - t0 >= args.duration:
                break
            continue

        vis = frame.copy()
        if det is not None:
            u = int(CX + det.offset * CX)
            cv2.line(vis, (u, int(IMG_H * ROI_TOP)), (u, IMG_H), (0, 200, 255), 2)
        hud(vis, [
            f"det={'OK' if det else 'LOST'} conf={det.confidence:.2f}" if det else "det=LOST",
            f"offset={det.offset:+.2f} curve={det.curve:+.2f}" if det else "",
            f"cmd v={out.v:.2f} w={out.w:+.2f}",
        ])
        cv2.imshow("lane demo", vis)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break

    cap.release()
    elapsed = time.monotonic() - t0
    print(f"\n处理 {frames} 帧，{elapsed:.1f}s，{frames / max(elapsed, 1e-6):.1f} fps")
    print(f"丢线帧 {lost} / {frames}（{100.0 * lost / max(frames, 1):.1f}%）")
    if offsets:
        a = np.array(offsets)
        print(f"offset: 平均 {a.mean():+.3f} 标准差 {a.std():.3f} 范围 [{a.min():+.2f}, {a.max():+.2f}]")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--synthetic", action="store_true", help="合成画面闭环（默认）")
    ap.add_argument("--camera", type=int, default=None, help="摄像头编号")
    ap.add_argument("--video", default=None, help="视频文件")
    ap.add_argument("--v-cruise", type=float, default=0.30)
    ap.add_argument("--fixed", action="store_true", help="改用固定阈值（默认自适应）")
    ap.add_argument("--threshold", type=int, default=90, help="固定阈值（未开自适应时生效）")
    ap.add_argument("--kp", type=float, default=1.30, help="循迹 P 增益")
    ap.add_argument("--kd", type=float, default=0.04, help="循迹 D 增益")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--duration", type=float, default=0.0, help="跑多少秒（合成模式）")
    ap.add_argument("--headless", action="store_true", help="不开窗口，只打印统计")
    ap.add_argument("--show-map", action="store_true", help="额外显示俯视图窗口")
    args = ap.parse_args()

    if not args.synthetic and args.camera is None and args.video is None:
        args.synthetic = True

    follower = LineFollower(LineConfig(v_cruise=args.v_cruise,
                                       kp=args.kp, kd=args.kd))
    detector = LineDetector(threshold=args.threshold, use_adaptive=not args.fixed)

    if args.synthetic:
        return run_synthetic(args, follower, detector)
    return run_image_source(args, follower, detector)


if __name__ == "__main__":
    raise SystemExit(main())
