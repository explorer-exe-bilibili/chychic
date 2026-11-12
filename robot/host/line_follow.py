"""循迹控制器：五路红外 + 视觉偏移融合，输出 (v, w)。

符号约定（全工程统一，改之前先看这里）

    w > 0  逆时针，车往左转
    w < 0  顺时针，车往右转
    error > 0  线在车的右边（视觉 offset 正 / 红外右侧触发）

所以 ``w = -PID(error)``。
"""

from __future__ import annotations

from dataclasses import dataclass, field

from pid import PID, RateLimiter

# 五路红外的位置权重，左 -> 右
IR_WEIGHTS = (-2.0, -1.0, 0.0, 1.0, 2.0)


@dataclass
class LineConfig:
    v_cruise: float = 0.35        # 直道巡航速度 m/s
    v_min: float = 0.10           # 转弯降速下限
    kp: float = 1.30
    kd: float = 0.04
    w_max: float = 1.80           # 角速度限幅 rad/s
    vision_gain: float = 0.70     # 视觉权重（有信心时）
    ir_gain: float = 0.30
    lost_speed_scale: float = 0.45  # 丢线时降到这个比例，慢慢找
    accel_max: float = 1.20       # m/s^2，输出限速
    turn_slow_gain: float = 0.55  # 转向越大越减速
    use_vision: bool = True


@dataclass
class LineOutput:
    v: float
    w: float
    error: float
    source: str  # "vision+ir" / "ir" / "lost"


@dataclass
class LineFollower:
    cfg: LineConfig = field(default_factory=LineConfig)

    def __post_init__(self) -> None:
        self._pid = PID(
            kp=self.cfg.kp,
            ki=0.0,
            kd=self.cfg.kd,
            out_min=-self.cfg.w_max,
            out_max=self.cfg.w_max,
        )
        self._limiter = RateLimiter(self.cfg.accel_max)
        self._last_ir_err = 0.0

    def reset(self) -> None:
        self._pid.reset()
        self._limiter.reset()
        self._last_ir_err = 0.0

    # ------------------------------------------------------------------
    @staticmethod
    def ir_error(ir: int) -> float | None:
        """五路红外 -> [-1, 1] 的归一化偏差。全黑/全白返回 None。"""
        active = [IR_WEIGHTS[i] for i in range(5) if ir & (1 << i)]
        if not active:
            return None
        return (sum(active) / len(active)) / 2.0

    # ------------------------------------------------------------------
    def update(
        self,
        ir: int,
        vision_offset: float | None,
        dt: float,
        vision_conf: float = 0.0,
    ) -> LineOutput:
        """vision_offset: [-1,1]，正 = 线在图像右侧；None 表示这一帧没看到线。"""
        ir_err = self.ir_error(ir)
        have_vision = (
            self.cfg.use_vision
            and vision_offset is not None
            and vision_conf >= 0.35
        )

        if ir_err is None and not have_vision:
            # 丢线：保持上次偏差，减速
            err = self._last_ir_err
            source = "lost"
            speed_scale = self.cfg.lost_speed_scale
        elif have_vision and ir_err is not None:
            err = (
                self.cfg.vision_gain * vision_offset
                + self.cfg.ir_gain * ir_err
            )
            source = "vision+ir"
            speed_scale = 1.0
        elif have_vision:
            err = vision_offset
            source = "vision"
            speed_scale = 1.0
        else:
            err = ir_err  # type: ignore[assignment]
            source = "ir"
            speed_scale = 1.0

        if ir_err is not None:
            self._last_ir_err = ir_err

        w = -self._pid.update(err, dt)

        # 转弯降速：|err| 越大越慢，出弯自动加速
        v_target = self.cfg.v_cruise * speed_scale * (
            1.0 - self.cfg.turn_slow_gain * min(1.0, abs(err))
        )
        v_target = max(self.cfg.v_min, v_target)
        v = self._limiter.update(v_target, dt)

        return LineOutput(v=v, w=w, error=err, source=source)
