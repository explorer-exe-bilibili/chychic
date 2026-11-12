"""跑一次、留一份档。

调参 90% 的时间花在"刚才到底发生了什么"，所以：
  - 每个控制周期一行 CSV（含红外、视觉、指令、实际反馈）
  - 事件另存一份时间线，方便对照
  - 可选录视频，和 CSV 用同一个相对时间轴

现场出问题，翻 logs/ 就行，不用靠回忆。
"""

from __future__ import annotations

import csv
import time
from pathlib import Path


class RunLogger:
    def __init__(self, outdir: str | Path = "logs", name: str = "run", video: bool = False) -> None:
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.dir = Path(outdir)
        self.dir.mkdir(parents=True, exist_ok=True)
        base = self.dir / f"{name}-{stamp}"

        self._csv_path = base.with_suffix(".csv")
        self._evt_path = base.with_suffix(".events.log")
        self._csv_file = open(self._csv_path, "w", newline="", encoding="utf-8")
        self._evt_file = open(self._evt_path, "w", encoding="utf-8")
        self._writer: csv.DictWriter | None = None
        self._fields: list[str] = []
        self._rows = 0
        self.t0 = time.monotonic()

        self._video = None
        if video:
            import cv2

            self._video = cv2.VideoWriter(
                str(base.with_suffix(".mp4")),
                cv2.VideoWriter_fourcc(*"mp4v"),
                20.0,
                (640, 480),
            )

    # ------------------------------------------------------------------
    def event(self, tag: str, msg: str) -> None:
        t = time.monotonic() - self.t0
        line = f"{t:8.3f}  [{tag:<6}] {msg}"
        self._evt_file.write(line + "\n")
        self._evt_file.flush()
        print(line, flush=True)

    def row(self, **kwargs) -> None:
        if self._writer is None:
            self._fields = list(kwargs.keys())
            self._writer = csv.DictWriter(self._csv_file, fieldnames=self._fields)
            self._writer.writeheader()
        for k in kwargs:
            if k not in self._fields:
                raise KeyError(f"日志字段 {k} 未在首行出现，保持一致")
        self._writer.writerow(kwargs)
        self._rows += 1
        if self._rows % 50 == 0:
            self._csv_file.flush()

    def frame(self, frame_bgr) -> None:
        if self._video is not None:
            self._video.write(frame_bgr)

    def close(self) -> None:
        self._csv_file.close()
        self._evt_file.close()
        if self._video is not None:
            self._video.release()

    # ------------------------------------------------------------------
    def __enter__(self) -> "RunLogger":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
