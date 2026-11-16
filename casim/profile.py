"""发动机转速曲线（虚拟飞轮的输入）。"""
from __future__ import annotations

import numpy as np


class SpeedProfile:
    """基类：输入时间采样 t（秒），输出转速 rpm（带符号，正=前进）。"""

    def speed(self, t: np.ndarray) -> np.ndarray:
        raise NotImplementedError


class ConstantSpeed(SpeedProfile):
    def __init__(self, rpm: float):
        self.rpm = float(rpm)

    def speed(self, t: np.ndarray) -> np.ndarray:
        return np.full(np.shape(t), self.rpm, dtype=float)


class RampSpeed(SpeedProfile):
    """从 start_rpm 以 accel [rpm/s] 线性加速到 target_rpm。

    cycle=True 时在 target_rpm 与 return_rpm 之间往复摆动
    （对应 CA Generator 的 Acceleration / 加速循环模式）。
    """

    def __init__(self, start_rpm: float = 0.0, target_rpm: float = 60.0,
                 accel: float = 10.0, cycle: bool = False,
                 return_rpm: float = 0.0):
        self.start = float(start_rpm)
        self.target = float(target_rpm)
        self.accel = float(accel)
        self.cycle = cycle
        self.ret = float(return_rpm)

    def speed(self, t: np.ndarray) -> np.ndarray:
        t = np.asarray(t, dtype=float)
        a = abs(self.accel) or 1e-9
        if not self.cycle:
            return np.clip(self.start + self.accel * t,
                           min(self.start, self.target),
                           max(self.start, self.target))
        d_up = (self.target - self.start) / a
        d_down = (self.target - self.ret) / a
        period = d_up + d_down
        tc = np.mod(t, period)
        up = self.start + self.accel * np.minimum(tc, d_up)
        down = self.target - self.accel * np.clip(tc - d_up, 0.0, d_down)
        return np.where(tc <= d_up, up, down)


class Fluctuation(SpeedProfile):
    """在基础转速上叠加正弦波动（模拟海浪 / 缸内燃烧的转速脉动）。"""

    def __init__(self, base: SpeedProfile, amplitude_rpm: float, freq_hz: float):
        self.base = base
        self.amp = float(amplitude_rpm)
        self.freq = float(freq_hz)

    def speed(self, t: np.ndarray) -> np.ndarray:
        t = np.asarray(t, dtype=float)
        return self.base.speed(t) + self.amp * np.sin(2.0 * np.pi * self.freq * t)


class CurveSpeed(SpeedProfile):
    """任意转速曲线：由 (时间 s, 转速 rpm) 折点线性插值。"""

    def __init__(self, t: np.ndarray, rpm: np.ndarray):
        self.t = np.asarray(t, dtype=float)
        self.rpm = np.asarray(rpm, dtype=float)

    def speed(self, t: np.ndarray) -> np.ndarray:
        return np.interp(np.asarray(t, dtype=float), self.t, self.rpm)
