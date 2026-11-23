"""摄像头：独立线程抓帧，主循环永远拿最新一帧，绝不阻塞。

算法侧最怕 ``cap.read()`` 卡住——一旦卡 2 秒，下位机看门狗就刹车了。
"""

from __future__ import annotations

import threading
import time

import numpy as np


class Camera:
    """摄像头源。

    source 支持三种写法：

        Camera(0)                                    # 本机 USB 相机 / 树莓派 CSI
        Camera("http://127.0.0.1:8080/video")        # MJPEG 流（手机当主控时用这个）
        Camera("/dev/video0")                        # Linux V4L2 设备节点

    手机方案下，内置摄像头由 App 在本地开 MJPEG 服务，OpenCV 直接读 HTTP，
    不经过 USB，绕开 Android 内核没有 UVC 驱动的问题。
    代价是延迟（典型 60-120ms），所以循迹 PID 应该放在下位机跑。
    """

    def __init__(
        self,
        source: int | str = 0,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        fourcc: str | None = None,
    ) -> None:
        import cv2  # 延迟导入

        self._cv2 = cv2
        self.source = source
        self.cap = cv2.VideoCapture(source)
        if not self.cap.isOpened():
            raise RuntimeError(f"打不开摄像头 source={source!r}")

        # 网络流不支持这些属性，设了也无害（返回 False）
        if not isinstance(source, str) or source.startswith("/dev/"):
            if fourcc:
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 关键：别让缓冲区堆积
        else:
            # MJPEG 流：让 OpenCV 用 FFMPEG 后端并尽量少缓冲
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

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
