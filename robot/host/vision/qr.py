"""启动二维码识别与任务解析。

二维码内容形如 ``123+123``：

  第一段 3 位 = 抓取顺序的颜色   1=绿色  2=深蓝色  3=浅蓝色
  第二段 3 位 = 对应物料放置的圆环号

⚠️ **规则手册里的例子和文字描述对不上**，必须去「规则答疑中心」确认：

  手册原文：「123+123 意为在 B 区先抓取绿色、再深蓝、再浅蓝；在 C 区把绿色
  放 3 号圆环、深蓝放 2 号圆环、浅蓝放 1 号圆环。」

  按位置对应应该是 绿→1、深蓝→2、浅蓝→3，但文字写的是反过来的。
  所以这里做成可配置项 ``RING_ORDER``，默认按**文字描述**（倒序），
  拿到官方答复后改一个常量即可，不影响其他代码。
"""

from __future__ import annotations

import re
from dataclasses import dataclass

from fsm import Task

COLOR_MAP = {"1": "green", "2": "darkblue", "3": "lightblue"}
COLOR_CN = {"green": "绿色", "darkblue": "深蓝色", "lightblue": "浅蓝色"}

# "reversed" = 按手册文字描述（第二段从后往前对应）
# "direct"   = 按位置对应
RING_ORDER = "reversed"


def parse_task(text: str) -> Task | None:
    """把二维码字符串解析成 Task。解析不了返回 None，调用方继续扫。"""
    text = (text or "").strip()
    if not text:
        return None

    groups = re.findall(r"[1-3]{3}", text)
    if len(groups) < 2:
        return None

    color_part, ring_part = groups[0], groups[1]
    colors = [COLOR_MAP.get(ch, "?") for ch in color_part]
    rings = [int(ch) for ch in ring_part]
    if "?" in colors:
        return None
    if RING_ORDER == "reversed":
        rings = list(reversed(rings))

    return Task(raw=text, colors=colors, rings=rings)


class QRReader:
    """OpenCV 自带 QRCodeDetector，不依赖 pyzbar（arm64 上装 zbar 挺麻烦）。"""

    def __init__(self, downscale: float = 1.0) -> None:
        import cv2

        self._cv2 = cv2
        self._det = cv2.QRCodeDetector()
        self._scale = downscale

    def read(self, frame_bgr) -> str | None:
        cv2 = self._cv2
        img = frame_bgr
        if self._scale != 1.0:
            img = cv2.resize(img, None, fx=self._scale, fy=self._scale)
        try:
            text, _pts, _ = self._det.detectAndDecode(img)
        except cv2.error:
            return None
        return text or None
