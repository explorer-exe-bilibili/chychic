"""里程计：编码器计数 -> 车体位姿 (x, y, theta)。

差速底盘，世界系 x 向前、y 向左、theta 逆时针为正（和 line_follow 的 w 一致）。

精度说明：这是"航迹推算"，误差会累积。1% 的轮径误差，走 10 米就差 10 厘米。
所以它只能用来做**短距离**参考（比如"再往前 300mm"），
绝不能用它去做 C 区 ±7mm 的最终对位——那一步必须靠视觉闭环。
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def __str__(self) -> str:
        return (
            f"x={self.x * 1000:+7.1f}mm "
            f"y={self.y * 1000:+7.1f}mm "
            f"th={math.degrees(self.theta):+6.1f}deg"
        )


class Odometry:
    def __init__(self, wheel_base: float, ticks_per_meter: float) -> None:
        if wheel_base <= 0 or ticks_per_meter <= 0:
            raise ValueError("wheel_base / ticks_per_meter 必须是正数")
        self.b = wheel_base
        self.tpm = ticks_per_meter
        self.pose = Pose()
        self._prev_l: int | None = None
        self._prev_r: int | None = None
        # 轮子方向：装配反了就把对应的一项改成 -1
        self.sign_l = 1
        self.sign_r = 1

    def reset(self) -> None:
        self.pose = Pose()
        self._prev_l = None
        self._prev_r = None

    def update(self, enc_l: int, enc_r: int) -> Pose:
        """喂绝对计数，返回新位姿。第一次调用只记录基准，不产生位移。"""
        if self._prev_l is None or self._prev_r is None:
            self._prev_l, self._prev_r = enc_l, enc_r
            return self.pose

        d_l = (enc_l - self._prev_l) * self.sign_l / self.tpm
        d_r = (enc_r - self._prev_r) * self.sign_r / self.tpm
        self._prev_l, self._prev_r = enc_l, enc_r

        ds = 0.5 * (d_l + d_r)
        dth = (d_r - d_l) / self.b

        # 用中点朝向积分，比用起点朝向准一档
        th_mid = self.pose.theta + 0.5 * dth
        self.pose.x += ds * math.cos(th_mid)
        self.pose.y += ds * math.sin(th_mid)
        self.pose.theta += dth

        # 归一化到 [-pi, pi]，避免长时间跑之后数值膨胀
        if self.pose.theta > math.pi:
            self.pose.theta -= 2 * math.pi
        elif self.pose.theta < -math.pi:
            self.pose.theta += 2 * math.pi
        return self.pose

    @property
    def distance(self) -> float:
        return math.hypot(self.pose.x, self.pose.y)
