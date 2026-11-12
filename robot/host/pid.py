"""带抗积分饱和与微分低通的 PID。

比赛现场第一课：PD 就够，I 项只在长距离直道有一点用；
加了 I 又不限幅，车会在原地慢慢抽。
"""

from __future__ import annotations


class PID:
    def __init__(
        self,
        kp: float,
        ki: float = 0.0,
        kd: float = 0.0,
        out_min: float = -1e9,
        out_max: float = 1e9,
        i_max: float = 0.0,
        d_lpf: float = 0.0,
    ) -> None:
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.i_max = abs(i_max)  # 0 表示不限
        self.d_lpf = d_lpf  # 0 表示不做低通
        self._i = 0.0
        self._prev_err: float | None = None
        self._prev_d = 0.0

    def reset(self) -> None:
        self._i = 0.0
        self._prev_err = None
        self._prev_d = 0.0

    def update(self, err: float, dt: float) -> float:
        if dt <= 1e-6:
            return 0.0

        p = self.kp * err

        d = 0.0
        if self._prev_err is not None:
            d = self.kd * (err - self._prev_err) / dt
            if self.d_lpf > 0.0:
                a = self.d_lpf
                d = a * self._prev_d + (1.0 - a) * d
        self._prev_d = d
        self._prev_err = err

        out = p + self._i + d
        # 饱和后再积分（clamping anti-windup）
        saturated = out > self.out_max or out < self.out_min
        if not saturated:
            self._i += self.ki * err * dt
            if self.i_max:
                self._i = max(-self.i_max, min(self.i_max, self._i))

        out = p + self._i + d
        return max(self.out_min, min(self.out_max, out))

    def __repr__(self) -> str:
        return f"PID(kp={self.kp}, ki={self.ki}, kd={self.kd}, i={self._i:+.4f})"


class RateLimiter:
    """把目标值限速，避免指令阶跃把车甩出去。"""

    def __init__(self, max_rate: float) -> None:
        self.max_rate = abs(max_rate)
        self._value = 0.0

    def reset(self, value: float = 0.0) -> None:
        self._value = value

    def update(self, target: float, dt: float) -> float:
        if dt <= 1e-6:
            return self._value
        step = self.max_rate * dt
        delta = target - self._value
        delta = max(-step, min(step, delta))
        self._value += delta
        return self._value
