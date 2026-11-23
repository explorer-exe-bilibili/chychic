"""全车硬件参数 —— **实测后只改这一个文件**。

这里的值同时被上位机和（人工同步到）下位机使用，两边必须一致：

    params.WHEEL_BASE  ==  firmware/App/Src/app.c 的 APP_WHEEL_BASE
    params.TICKS_PER_METER == firmware/App/Src/app.c 的 APP_TICKS_PER_METER

不标定就上场，等于闭着眼睛调 PID。
标定流程：``python3 tools/bringup.py --port /dev/ttyUSB0``
"""

from __future__ import annotations

import json
from pathlib import Path

# ==========================================================================
#  待实测（先用模型值占位，量完改成实测值）
# ==========================================================================

#: 轮距，左右驱动轮接地点中心距（米）
#: 模型 x=±126 → 0.252；换 80mm 橡胶轮后必须重新量
WHEEL_BASE = 0.252

#: 轮子转 1 米对应的编码器计数（4 倍频累计）
#: 计算公式：PPR * 4 * 减速比 / (pi * 轮径)
#: 例：霍尔 11 线、4 倍频、减速比 30、轮径 0.080
#:     = 11*4*30 / (3.14159*0.080) = 1320 / 0.2513 = 5253
#: 这个值必须用 bringup.py 的"推车一米"法实测，公式值只作参考
TICKS_PER_METER = 5253.0

#: 轮径（米），换轮时才改
WHEEL_DIAMETER = 0.080

#: 相机安装参数（标定后填）
#: 高度：镜头光心距地面（米）
CAM_HEIGHT = 0.110
#: 俯角：光轴与水平面夹角（度），向前下方为正
CAM_PITCH_DEG = 40.0

# ==========================================================================
#  循迹由谁做 —— 两种架构共用一套代码，只切这一个开关
# ==========================================================================
#: True  : MCU 自主循迹（app_line_follow），上位机只切模式，不插手速度
#: False : 上位机做循迹（摄像头方案），MANUAL 模式下每帧下发 (v, w)
#:
#: 五路红外不好用、改摄像头循迹时把这里改成 False 即可，其他代码不用动。
LINE_FOLLOW_ON_MCU = True

# ==========================================================================
#  控制参数（地面实测调，不用改上面的）
# ==========================================================================

#: 直道巡航速度 m/s。从 0.25 起，稳了再往上加
V_CRUISE = 0.30

#: 下位机看门狗超时，必须和固件 APP_WDOG_TIMEOUT_MS 一致（秒）
WDOG_TIMEOUT = 0.200


def as_dict() -> dict:
    return {
        "WHEEL_BASE": WHEEL_BASE,
        "TICKS_PER_METER": TICKS_PER_METER,
        "WHEEL_DIAMETER": WHEEL_DIAMETER,
        "CAM_HEIGHT": CAM_HEIGHT,
        "CAM_PITCH_DEG": CAM_PITCH_DEG,
        "V_CRUISE": V_CRUISE,
    }


def load_override(path: str | Path = "params.json") -> dict:
    """如果同目录有 params.json，用它的值覆盖（不改代码就能换车）。"""
    p = Path(path)
    if not p.exists():
        return {}
    data = json.loads(p.read_text(encoding="utf-8"))
    g = globals()
    for k, v in data.items():
        if k in g:
            g[k] = v
    return data
