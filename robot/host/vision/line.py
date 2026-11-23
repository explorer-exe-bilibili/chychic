"""赛道黑线检测。

策略：只看画面下半部分的 ROI，按行取暗像素加权质心，取底部三行的平均作为
横向偏差，顶部几行给出曲率提示（用于提前转向）。

比"整幅图找最大轮廓"稳，因为场地边缘、影子、其他队伍的道具不会干扰底部 ROI。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Detection:
    offset: float       # [-1, 1]，正 = 线在画面右侧
    confidence: float   # [0, 1]
    curve: float        # [-1, 1]，正 = 赛道向右弯
    rows: int           # 实际参与计算的行数


class LineDetector:
    def __init__(
        self,
        roi_top: float = 0.55,
        threshold: int = 90,
        use_adaptive: bool = False,
        min_pixels: int = 25,
    ) -> None:
        self.roi_top = roi_top
        self.threshold = threshold
        self.use_adaptive = use_adaptive
        self.min_pixels = min_pixels

    def _mask(self, gray: np.ndarray, cv2) -> np.ndarray:
        if self.use_adaptive:
            mask = cv2.adaptiveThreshold(
                gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 31, 10
            )
        else:
            _, mask = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY_INV)
        # 去掉细小噪点
        kernel = np.ones((3, 3), np.uint8)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    @staticmethod
    def _row_centroid(mask: np.ndarray, row: np.ndarray, min_pixels: int) -> float | None:
        idx = np.flatnonzero(row)
        if idx.size < min_pixels:
            return None
        weights = row[idx].astype(np.float32)
        return float((idx * weights).sum() / weights.sum())

    def detect(self, frame_bgr: np.ndarray) -> Detection | None:
        import cv2

        h, w = frame_bgr.shape[:2]
        y0 = int(h * self.roi_top)
        roi = frame_bgr[y0:, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        mask = self._mask(gray, cv2)

        rh = mask.shape[0]
        half = w / 2.0

        # 底部 20% 的几行 -> 当前偏差
        near_rows = range(int(rh * 0.80), rh, max(1, rh // 60))
        # 顶部 20% 的几行 -> 曲率
        far_rows = range(int(rh * 0.05), int(rh * 0.30), max(1, rh // 60))

        near = [c for r in near_rows if (c := self._row_centroid(mask, mask[r], self.min_pixels))]
        far = [c for r in far_rows if (c := self._row_centroid(mask, mask[r], self.min_pixels))]

        if not near:
            return None

        cx_near = float(np.median(near))
        offset = (cx_near - half) / half
        offset = float(np.clip(offset, -1.0, 1.0))

        curve = 0.0
        if far:
            cx_far = float(np.median(far))
            curve = float(np.clip((cx_far - cx_near) / half, -1.0, 1.0))

        pixels = int(np.count_nonzero(mask[-int(rh * 0.25):, :]))
        conf = float(np.clip(pixels / 400.0, 0.0, 1.0))

        return Detection(offset=offset, confidence=conf, curve=curve, rows=len(near))
