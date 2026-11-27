"""MCU 高层接口：把协议细节挡在这一层之下。

## 为什么这样分层

读了电控代码（`app/Src/app_line_follow.c`）之后，架构前提变了：

- **循迹 PID 在 STM32 里**，MCU 自己采样五路红外、自己算角速度、自己处理
  丢线 / 宽线 / 多线段，上位机**不下发速度**。
- 所以上位机绝大多数时间只需要"切模式"，不需要 50Hz 喂指令。
- 只有 A 区绕方块、C 区对中这类"脱离黑线"的动作，才切到 MANUAL 下发速度。

于是上层只认四个动作：

    mcu.start_line_follow()   # 交给 MCU 自己循迹
    mcu.manual(v_m_s, w)      # 我接管，按我给的速度走
    mcu.stop()                # 停
    mcu.estop()               # 刹车锁死

## 单位约定（踩过一次坑，写死在这）

- 上位机内部：**m/s**、rad/s
- 线路上（和 MCU 的 chassis 接口）：**mm/s**、rad/s
- 换算只在 `link.SerialLink.send_vel` 一处做，别到处乘 1000
"""

from __future__ import annotations

import time

import protocol as p

#: 看门狗续命频率（Hz）。MCU 侧建议 300ms 超时，20Hz 有足够余量。
KEEPALIVE_HZ = 20.0


class MCU:
    def __init__(self, link, wheel_base_m: float = 0.234) -> None:
        self.link = link
        self.wheel_base = wheel_base_m
        self.state: p.State | None = None
        self.mode = p.MODE_IDLE
        self._last_ping = 0.0
        self._last_rx = 0.0

    # -- 收发 ---------------------------------------------------------
    def poll(self) -> list[p.State]:
        states = self.link.poll()
        if states:
            self.state = states[-1]
            self._last_rx = time.monotonic()
        return states

    def is_online(self, timeout: float = 0.5) -> bool:
        """最近 timeout 秒内是否收到过遥测。没收到说明链路或下位机有问题。"""
        return self.state is not None and (time.monotonic() - self._last_rx) <= timeout

    def keepalive(self, now: float | None = None) -> None:
        """每个主循环都要调。MCU 靠它在主机挂掉时判断失联。"""
        t = time.monotonic() if now is None else now
        if t - self._last_ping >= 1.0 / KEEPALIVE_HZ:
            self.link.send_ping()
            self._last_ping = t

    # -- 模式 ---------------------------------------------------------
    def set_mode(self, mode: int) -> None:
        if mode != self.mode:
            self.link.send_mode(mode)
            self.mode = mode

    def start_line_follow(self) -> None:
        """把控制权交给 MCU 的循迹模块。"""
        self.set_mode(p.MODE_LINE_FOLLOW)

    def manual(self, v_m_s: float, w_rad_s: float) -> None:
        """接管控制：先切进 MANUAL，再按上位机的速度走。

        注意每帧都要调，MANUAL 模式下 MCU 不自己产生速度。
        """
        self.set_mode(p.MODE_MANUAL)
        self.link.send_vel(v_m_s, w_rad_s)

    def stop(self) -> None:
        self.set_mode(p.MODE_IDLE)

    def estop(self) -> None:
        self.link.estop()
        self.mode = p.MODE_ESTOP

    # -- 升降台 -------------------------------------------------------
    @property
    def lift(self):
        """最近一帧升降台状态；没收到过则为 None。"""
        return getattr(self.link, "lift", None)

    def lift_move(self, target_mm: int) -> None:
        """移动到指定高度（毫米）。未标定零点前 position 不可信。"""
        self.link.send_raw(lambda s: p.encode_lift_move(target_mm, s))

    def lift_stop(self) -> None:
        self.link.send_raw(p.encode_lift_stop)

    def lift_set_zero(self) -> None:
        """把当前位置记作零点。换丝杆 / 换电机后必须重做。"""
        self.link.send_raw(p.encode_lift_set_zero)

    def lift_config(self, counts_per_mm: float, max_height_mm: int) -> None:
        self.link.send_raw(lambda s: p.encode_lift_config(counts_per_mm, max_height_mm, s))

    # -- 查询 ---------------------------------------------------------
    @property
    def lf_state(self) -> int:
        return self.state.lf_state if self.state else p.LF_DISABLED

    @property
    def following(self) -> bool:
        return self.lf_state in (p.LF_ACQUIRING, p.LF_TRACKING)

    @property
    def lost(self) -> bool:
        return self.lf_state in (p.LF_LOST, p.LF_WIDE_LINE, p.LF_AMBIGUOUS)

    @property
    def healthy(self) -> bool:
        """有没有在正常回传，且没进看门狗/急停。"""
        return self.state is not None and not (
            self.state.watchdog_tripped or self.state.estop
        )

    def describe(self) -> str:
        if self.state is None:
            return "MCU(未连接)"
        return f"MCU[{self.state.describe()}]"
