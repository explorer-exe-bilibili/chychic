"""摄像头：独立线程抓帧，主循环永远拿最新一帧，绝不阻塞。

算法侧最怕 ``cap.read()`` 卡住——一旦卡 2 秒，下位机看门狗就刹车了。
"""

from __future__ import annotations

import threading
import time

import numpy as np


class Camera:
    def __init__(
        self,
        index: int = 0,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        fourcc: str | None = None,
    ) -> None:
        import cv2  # 延迟导入

        self._cv2 = cv2
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError(f"打不开摄像头 index={index}")
        if fourcc:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 关键：别让缓冲区堆积

        self._frame: np.ndarray | None = None
        self._seq = 0
        self._alive = True
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._loop, name="camera", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while self._alive:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.005)
                continue
            with self._lock:
                self._frame = frame
                self._seq += 1

    def read(self) -> tuple[np.ndarray | None, int]:
        """返回 (最新帧, 帧序号)。帧序号用来判断视觉是否卡住。"""
        with self._lock:
            if self._frame is None:
                return None, 0
            return self._frame.copy(), self._seq

    def close(self) -> None:
        self._alive = False
        self._thread.join(timeout=1.0)
        self.cap.release()
