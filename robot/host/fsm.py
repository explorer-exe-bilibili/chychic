"""任务状态机：启停区 → A → B → C → D → 启停区。

设计原则（比代码本身重要）

1. 每个状态都有**超时**，超时后**降级进入下一状态**而不是停车死等。
   赛场上卡住 = 这轮 0 分；跑完 60% = 照样有分。
2. 任何异常都进 ``FAILSAFE``，刹车并记录。
3. 状态切换只做三件事：打日志、重置控制器、记录时刻。
"""

from __future__ import annotations

import enum
import time
from dataclasses import dataclass, field
from typing import Callable

from line_follow import LineFollower, LineOutput


class S(enum.Enum):
    BOOT = enum.auto()
    IDLE = enum.auto()
    WAIT_QR = enum.auto()
    SEG_A = enum.auto()
    SEG_B = enum.auto()
    SEG_C = enum.auto()
    SEG_D = enum.auto()
    RETURN = enum.auto()
    DONE = enum.auto()
    FAILSAFE = enum.auto()


# 每个状态的超时（秒）。按 2.4m 场地、0.35m/s 巡航估的，实测后调。
TIMEOUTS: dict[S, float] = {
    S.BOOT: 2.0,
    S.IDLE: 1e9,
    S.WAIT_QR: 20.0,
    S.SEG_A: 30.0,
    S.SEG_B: 40.0,
    S.SEG_C: 40.0,
    S.SEG_D: 35.0,
    S.RETURN: 20.0,
    S.DONE: 1e9,
    S.FAILSAFE: 1e9,
}


@dataclass
class Task:
    """二维码解析结果。"""

    raw: str = ""
    colors: list[str] = field(default_factory=list)   # 抓取顺序
    rings: list[int] = field(default_factory=list)    # 放置环号

    def __str__(self) -> str:
        return f"Task({self.raw!r} colors={self.colors} rings={self.rings})"


class Mission:
    def __init__(self, follower: LineFollower, log) -> None:
        self.follower = follower
        self.log = log
        self.state = S.BOOT
        self.t_enter = time.monotonic()
        self.task = Task()
        self.last: LineOutput | None = None
        self._handlers: dict[S, Callable[[float], None]] = {
            S.SEG_A: self._step_line,
            S.SEG_B: self._step_pass_through,
            S.SEG_C: self._step_pass_through,
            S.SEG_D: self._step_line,
            S.RETURN: self._step_line,
        }

    # -- 状态迁移 -------------------------------------------------------
    def goto(self, new: S) -> None:
        if new is self.state:
            return
        self.log.event("FSM", f"{self.state.name} -> {new.name}")
        self.state = new
        self.t_enter = time.monotonic()
        self.follower.reset()

    @property
    def elapsed(self) -> float:
        return time.monotonic() - self.t_enter

    def timed_out(self) -> bool:
        return self.elapsed > TIMEOUTS.get(self.state, 1e9)

    def fail(self, why: str) -> None:
        self.log.event("FSM", f"FAILSAFE: {why}")
        self.state = S.FAILSAFE
        self.t_enter = time.monotonic()

    # -- 每帧 -----------------------------------------------------------
    def step(self, ir: int, vision_offset: float | None, vision_conf: float, dt: float) -> LineOutput:
        """返回本帧应该下发的速度指令。"""
        self.last = self._run(ir, vision_offset, vision_conf, dt)
        return self.last

    def _run(self, ir: int, vision_offset: float | None, vision_conf: float, dt: float) -> LineOutput:
        if self.state is S.BOOT:
            self.goto(S.IDLE)
        if self.state is S.IDLE:
            return LineOutput(0.0, 0.0, 0.0, "idle")
        if self.state is S.FAILSAFE:
            return LineOutput(0.0, 0.0, 0.0, "failsafe")
        if self.state is S.DONE:
            return LineOutput(0.0, 0.0, 0.0, "done")

        if self.timed_out():
            self.log.event("FSM", f"{self.state.name} 超时 {TIMEOUTS[self.state]:.0f}s，降级")
            self.goto(self._next_after(self.state))

        handler = self._handlers.get(self.state)
        if handler is None:
            return LineOutput(0.0, 0.0, 0.0, self.state.name.lower())
        return handler(ir, vision_offset, vision_conf, dt)

    # -- 各状态的具体行为 ------------------------------------------------
    def _step_line(self, ir, vision_offset, vision_conf, dt) -> LineOutput:
        return self.follower.update(ir, vision_offset, dt, vision_conf)

    def _step_pass_through(self, ir, vision_offset, vision_conf, dt) -> LineOutput:
        """B/C 区当前是占位：先按循迹跑过去。

        接上抓取/放置后，在这里判断"完成 N 个物料"再 goto 下一状态。
        """
        return self.follower.update(ir, vision_offset, dt, vision_conf)

    @staticmethod
    def _next_after(state: S) -> S:
        """超时/完成后去哪。B、C 失败也要继续往下跑。"""
        return {
            S.WAIT_QR: S.SEG_A,
            S.SEG_A: S.SEG_B,
            S.SEG_B: S.SEG_C,
            S.SEG_C: S.SEG_D,
            S.SEG_D: S.RETURN,
            S.RETURN: S.DONE,
        }.get(state, S.FAILSAFE)

    def on_qr(self, task: Task) -> None:
        self.task = task
        self.log.event("QR", str(task))
        self.goto(S.SEG_A)
