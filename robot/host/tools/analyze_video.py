#!/usr/bin/env python3
"""用录好的视频评估循迹检测器。

    python3 tools/analyze_video.py --video 赛道.mp4
    python3 tools/analyze_video.py --video 赛道.mp4 --sheet out.png --samples 9

输出：
  - 终端里每帧的 offset / conf / 是否丢线
  - 一张抽帧对比图（检测线叠加在原图上），肉眼判断阈值合不合适
  - 整段视频的丢线率，决定固定阈值够不够用
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vision.line import LineDetector  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True)
    ap.add_argument("--sheet", default="lane_sheet.png", help="对比图输出路径")
    ap.add_argument("--samples", type=int, default=6, help="对比图抽几帧")
    ap.add_argument("--threshold", type=int, default=90, help="灰度阈值")
    ap.add_argument("--fixed", action="store_true", help="改用固定阈值（默认自适应）")
    ap.add_argument("--max-frames", type=int, default=0, help="只处理前 N 帧（0=全部）")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"打不开视频: {args.video}")
        return 1

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    limit = total if args.max_frames <= 0 else min(total, args.max_frames)

    print(f"视频 {w}x{h}  {fps:.1f}fps  {total} 帧（约 {total / fps:.1f}s）")
    print(f"检测器 阈值={args.threshold}  自适应={not args.fixed}\n")

    det = LineDetector(threshold=args.threshold, use_adaptive=not args.fixed)
    pick = {int(limit * i / max(1, args.samples - 1) / 1) for i in range(args.samples)}
    pick = {min(p, limit - 1) for p in pick}

    tiles = []
    total_n = lost = 0
    offsets = []
    confs = []

    for idx in range(limit):
        ok, frame = cap.read()
        if not ok:
            break
        total_n += 1
        r = det.detect(frame)

        if r is None:
            lost += 1
        else:
            offsets.append(r.offset)
            confs.append(r.confidence)

        if idx in pick:
            vis = frame.copy()
            y0 = int(h * det.roi_top)
            cv2.line(vis, (0, y0), (w, y0), (0, 160, 255), 1)
            if r is not None:
                u = int(w / 2 + r.offset * w / 2)
                cv2.line(vis, (u, y0), (u, h), (0, 200, 255), 2)
                txt = f"off={r.offset:+.2f} conf={r.confidence:.2f}"
            else:
                txt = "LOST"
            cv2.putText(vis, txt, (8, y0 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
            cv2.putText(vis, txt, (8, y0 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 1)
            cv2.putText(vis, f"#{idx}", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
            cv2.putText(vis, f"#{idx}", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            tw = 420
            tiles.append(cv2.resize(vis, (tw, int(tw * h / w))))

    cap.release()

    print(f"处理 {total_n} 帧，丢线 {lost} 帧（{100.0 * lost / max(total_n, 1):.1f}%）")
    if offsets:
        a = np.array(offsets)
        c = np.array(confs)
        print(f"offset  平均 {a.mean():+.3f}  标准差 {a.std():.3f}  "
              f"范围 [{a.min():+.2f}, {a.max():+.2f}]")
        print(f"confidence 平均 {c.mean():.2f}  低于 0.35 的比例 {100.0 * (c < 0.35).mean():.1f}%")

    if tiles:
        while len(tiles) % 3:
            tiles.append(np.zeros_like(tiles[0]))
        rows = [np.hstack(tiles[i:i + 3]) for i in range(0, len(tiles), 3)]
        sheet = np.vstack(rows)
        cv2.imwrite(args.sheet, sheet)
        print(f"\n对比图: {args.sheet}  ({sheet.shape[1]}x{sheet.shape[0]})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
