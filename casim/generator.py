"""CA 信号仿真主入口：转速曲线 -> 虚拟飞轮角度 -> 传感器/SSI 信号。"""
from __future__ import annotations

import os
import warnings
from dataclasses import dataclass

import numpy as np

from .profile import SpeedProfile, ConstantSpeed
from .sensor import SensorSpec
from .ssi import SSISpec

# NI 9474 接线 (2 组, 每组 4 路):
#   组 1: CH00 TDC1, CH01 REF1, CH02 A, CH03 B
#   组 2: CH04 REF2, CH05 C, CH06 D, CH07 TDC2
# TDC 由 2 个独立冗余传感器 TDC1/TDC2 提供 (各 1 路, 同读 1 个 TDC 标记)
WIRING = [
    ("CH00", "TDC1"),
    ("CH01", "REF1"),
    ("CH02", "A"),
    ("CH03", "B"),
    ("CH04", "REF2"),
    ("CH05", "C"),
    ("CH06", "D"),
    ("CH07", "TDC2"),
]


@dataclass
class SimResult:
    t: np.ndarray          # 时间 [s]
    angle: np.ndarray      # 曲轴角 [deg], 不取模 (虚拟飞轮)
    rpm: np.ndarray        # 转速 [rpm]
    signals: dict          # 名称 -> int8 数组 (A, B, C, D, REF1, REF2, TDC1, TDC2)
    ssi1: np.ndarray       # SSI 读字 (bit12 = PFB)
    ssi2: np.ndarray
    dt: float

    def channels(self) -> dict:
        """按 NI 9474 接线顺序的通道 dict: CH00..CH07 (CH00=TDC1, CH07=TDC2)。"""
        return {ch: self.signals[sig] for ch, sig in WIRING}

    def csv_columns(self):
        return (["t_s", "angle_deg", "speed_rpm"]
                + [ch for ch, _ in WIRING] + ["ssi1", "ssi2"])

    def to_csv(self, path: str):
        parent = os.path.dirname(os.path.abspath(path))
        os.makedirs(parent, exist_ok=True)
        ch = self.channels()
        cols = [self.t, self.angle, self.rpm]
        cols += [ch[c].astype(np.float64) for c in ch]
        cols += [self.ssi1.astype(np.float64), self.ssi2.astype(np.float64)]
        fmts = ["%.6f", "%.4f", "%.3f"] + ["%.0f"] * (len(WIRING) + 2)
        np.savetxt(path, np.column_stack(cols), delimiter=",",
                   header=",".join(self.csv_columns()), comments="", fmt=fmts)


class CASimulator:
    """由转速仿真曲轴角度传感器信号。

    用法:
        sim = CASimulator(ConstantSpeed(60.0), dt=50e-6, duration=5.0)
        res = sim.run()
        res.to_csv("out.csv")
    """

    def __init__(self, profile: SpeedProfile | None = None, *,
                 dt: float = 50e-6, duration: float = 5.0,
                 start_angle: float = 0.0,
                 sensors: SensorSpec | None = None,
                 ssi: SSISpec | None = None):
        self.profile = profile or ConstantSpeed(60.0)
        self.dt = float(dt)
        self.duration = float(duration)
        self.start_angle = float(start_angle)
        self.sensors = sensors or SensorSpec()
        self.ssi = ssi or SSISpec()

    def run(self) -> SimResult:
        t = np.arange(0.0, self.duration + 0.5 * self.dt, self.dt)
        rpm = np.asarray(self.profile.speed(t), dtype=float)

        # 虚拟飞轮: 梯形积分, d[deg] = rpm * 6 [deg/s] * dt
        inc = 0.5 * (rpm[1:] + rpm[:-1]) * 6.0 * self.dt
        angle = np.empty(t.shape, dtype=float)
        angle[0] = self.start_angle
        angle[1:] = self.start_angle + np.cumsum(inc)

        signals = self.sensors.signals(angle)
        ssi1, ssi2 = self.ssi.words(angle)

        # 采样分辨率检查: 最窄脉冲在最高转速下的持续时间
        min_width = min(self.sensors.a_width, self.sensors.b_width,
                        self.sensors.c_width, self.sensors.d_width,
                        self.sensors.ref1_width, self.sensors.ref2_width,
                        self.sensors.tdc1_width, self.sensors.tdc2_width)
        max_rpm = max(1e-9, float(np.max(np.abs(rpm))))
        min_pulse_us = min_width / (6.0 * max_rpm) * 1e6
        if self.dt * 1e6 >= min_pulse_us:
            warnings.warn(
                f"dt={self.dt * 1e6:.1f} us 可能过粗: 最窄脉冲 {min_width:.2f} deg "
                f"在 {max_rpm:.0f} rpm 下持续 {min_pulse_us:.1f} us")

        return SimResult(t=t, angle=angle, rpm=rpm, signals=signals,
                         ssi1=ssi1, ssi2=ssi2, dt=self.dt)
