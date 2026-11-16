"""拾波 (pickup) 传感器信号模型 + 传感器安装位置配置。

物理模型（对应飞轮安装图）：
- 飞轮齿轮（单飞轮单齿轮）有 `teeth` 个齿均布, A/B/C/D 四路拾波同读
  该齿轮 (共用齿数与齿位网格), 第 0 齿中心位于飞轮 `tooth0` 位置
  (deg, 相对飞轮 0°);
- 飞轮上加工有 REF1 / REF2 / TDC 标记, 位置 `ref1_mark` / `ref2_mark` /
  `tdc_mark` (deg, 相对飞轮 0°); TDC (第 1 缸上止点) 为唯一曲轴角,
  飞轮上仅 1 个 TDC 标记;
- 传感器 (PU A1, PU B1, REF1, PU C1, PU D1, REF2, TDC1, TDC2) 安装在
  发动机坐标系的固定角度 `pos_*` (deg, 与飞轮同 0° 基准, 正方向 = 发动机
  前进旋转方向); TDC1/TDC2 为两个冗余拾波传感器, 同读 1 个 TDC 标记
  (TDC1 → 通道组 1, TDC2 → 通道组 2);
- 齿/标记转到传感器下方时产生脉冲, 脉冲中心 (发动机角度) 为:
      齿拾波:  center = (pos - tooth0) mod (360/teeth)
      标记:    center = (pos - mark)  mod 360

脉冲为理想方波: 周期 T、宽度 L (*_width)、中心在 center。

信号模型公式（WinGD《CA Generator Model》文档）：
    y(th) = sin(b*th + c) + d,   信号 = (y > 0)
    b = 2*pi / T                      (T = 周期, deg)
    c = PUoffset - REF + T/4          (脉冲中心定位)
    d = -cos(b * L/2)                 (L = 高电平宽度, 与文档 Eq.29 联立的闭式解)

注: 文档 Eq.30 写作 d = -sin(b*(T/2-L/2))，与其 Eq.29
(L = T/2 - 2*arcsin(-d)/b) 联立解得 d = -cos(b*L/2)，
本实现采用该闭式解（L = T/2 时 d = 0，占空 50%）。

对正弦取阈值 (y>0) 后，输出即为理想方波：周期 T、宽度 L、
中心位于 center。直接方波实现 square_wave() 与其严格等价。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def square_wave(angle, period_deg: float, width_deg: float,
                center_deg: float) -> np.ndarray:
    """数字拾波信号。

    angle 落在 [center - width/2, center + width/2) (mod period) 时为 1。
    angle: 标量或数组，单位 deg（绝对曲轴角，可超过 360）。
    """
    angle = np.asarray(angle, dtype=float)
    rel = (angle - center_deg + 0.5 * width_deg) % period_deg
    return (rel < width_deg).astype(np.int8)


def square_wave_sine(angle, period_deg: float, width_deg: float,
                     center_deg: float) -> np.ndarray:
    """参考实现：CA Generator Model 文档的正弦参数化公式。

    与 square_wave() 等价，用于交叉验证 / LabVIEW 移植对照。
    """
    angle = np.asarray(angle, dtype=float)
    b = 2.0 * np.pi / period_deg
    # 令正弦峰值出现在 center: peak at (pi/2 - c)/b = center
    c = np.pi / 2.0 - b * center_deg
    # 高电平宽度 = L: d = -cos(b*L/2)  (见模块 docstring 推导)
    d = -np.cos(b * 0.5 * width_deg)
    return (np.sin(b * angle + c) + d > 0).astype(np.int8)


@dataclass
class SensorSpec:
    """传感器安装位置配置（物理模型），角度单位均为 deg。

    角度基准: 0° = 飞轮 0° (TDC = 0° 参考), 正方向 = 发动机前进旋转方向。

    安装位置配置分三类:
    1. 飞轮齿轮:   teeth (齿数, A/B/C/D 共用), tooth0 (第 0 齿中心位置)
    2. 飞轮标记:   ref1_mark / ref2_mark / tdc_mark (加工在飞轮上的位置)
    3. 传感器安装: pos_a / pos_b / pos_ref1 / pos_c / pos_d / pos_ref2 /
                   pos_tdc1 / pos_tdc2 (传感器在发动机坐标系中的固定安装角;
                   TDC1/TDC2 为两个冗余传感器, 同读 1 个 TDC 标记)
    脉冲宽度: a_width ~ tdc2_width (deg)
    """

    # ---- 飞轮齿轮 (单飞轮单齿轮, A/B/C/D 共用同一齿数与齿位网格) ----
    teeth: int = 100            # 齿轮齿数 (4 路拾波同读一个飞轮齿轮), 齿周期 = 360/teeth
    tooth0: float = 0.0         # 第 0 齿中心在飞轮上的位置 (deg)

    # ---- 飞轮标记位置 ----
    ref1_mark: float = 0.0      # REF1 标记在飞轮上的位置 (deg)
    ref2_mark: float = 0.0      # REF2 标记在飞轮上的位置 (deg)
    tdc_mark: float = 0.0       # TDC 标记在飞轮上的位置 (deg)

    # ---- 传感器安装位置 (发动机坐标系固定角, deg) ----
    pos_a: float = 0.9          # PU A1 传感器
    pos_b: float = 2.7          # PU B1 传感器 (A-B = pos_b - pos_a)
    pos_ref1: float = 0.0       # REF1 传感器
    pos_c: float = 0.9          # PU C1 传感器
    pos_d: float = 2.7          # PU D1 传感器 (C-D = pos_d - pos_c)
    pos_ref2: float = 0.0       # REF2 传感器
    pos_tdc1: float = 0.0       # TDC1 传感器 (通道组 1, CH00)
    pos_tdc2: float = 0.0       # TDC2 传感器 (通道组 2, CH07), 与 TDC1 冗余

    # ---- 脉冲宽度 (deg) ----
    a_width: float = 1.8
    b_width: float = 1.8
    c_width: float = 1.8
    d_width: float = 1.8
    ref1_width: float = 0.6
    ref2_width: float = 0.6
    tdc1_width: float = 0.5
    tdc2_width: float = 0.5

    @property
    def tooth_period(self) -> float:
        return 360.0 / self.teeth

    def centers(self) -> dict:
        """各传感器脉冲中心 (发动机角度, deg):
        齿拾波 center = (pos - tooth0) mod 齿周期; 标记 center = (pos - mark) mod 360。
        A/B/C/D 同读一个飞轮齿轮, 共用 tooth_period;
        TDC1/TDC2 同读 1 个 TDC 标记 (唯一曲轴角)。
        """
        P = self.tooth_period
        return {
            "A": (self.pos_a - self.tooth0) % P,
            "B": (self.pos_b - self.tooth0) % P,
            "REF1": (self.pos_ref1 - self.ref1_mark) % 360.0,
            "C": (self.pos_c - self.tooth0) % P,
            "D": (self.pos_d - self.tooth0) % P,
            "REF2": (self.pos_ref2 - self.ref2_mark) % 360.0,
            "TDC1": (self.pos_tdc1 - self.tdc_mark) % 360.0,
            "TDC2": (self.pos_tdc2 - self.tdc_mark) % 360.0,
        }

    def signals(self, angle) -> dict:
        """返回 A, B, C, D, REF1, REF2, TDC1, TDC2 的 int8 数组 dict。"""
        P = self.tooth_period
        c = self.centers()
        return {
            "A": square_wave(angle, P, self.a_width, c["A"]),
            "B": square_wave(angle, P, self.b_width, c["B"]),
            "REF1": square_wave(angle, 360.0, self.ref1_width, c["REF1"]),
            "C": square_wave(angle, P, self.c_width, c["C"]),
            "D": square_wave(angle, P, self.d_width, c["D"]),
            "REF2": square_wave(angle, 360.0, self.ref2_width, c["REF2"]),
            "TDC1": square_wave(angle, 360.0, self.tdc1_width, c["TDC1"]),
            "TDC2": square_wave(angle, 360.0, self.tdc2_width, c["TDC2"]),
        }

    @classmethod
    def from_offsets(cls, teeth: int = 100,
                     a_ref1_offset: float = 0.9, a_b_offset: float = 1.8,
                     c_ref2_offset: float = 0.9, c_d_offset: float = 1.8,
                     ref1_angle: float = 0.0, ref2_angle: float = 0.0,
                     tdc_angle: float = 0.0,
                     a_width: float = 1.8, b_width: float = 1.8,
                     c_width: float = 1.8, d_width: float = 1.8,
                     ref1_width: float = 0.6, ref2_width: float = 0.6,
                     tdc_width: float = 0.5) -> "SensorSpec":
        """按旧式 “脉冲中心偏移” 参数构造（等效配置, 便于迁移）。

        teeth: 飞轮齿轮齿数 (A/B/C/D 共用, 单飞轮单齿轮);
        a_ref1_offset: A 中心相对 REF1 中心; a_b_offset: B 中心相对 A 中心;
        ref1_angle/ref2_angle/tdc_angle: 各 REF/TDC 中心位置
        (tdc_angle/tdc_width 同时用于 TDC1/TDC2 两个冗余传感器)。
        内部取 mark = 0, pos = 对应中心角。
        """
        return cls(
            teeth=teeth, tooth0=0.0,
            ref1_mark=0.0, ref2_mark=0.0, tdc_mark=0.0,
            pos_a=ref1_angle + a_ref1_offset,
            pos_b=ref1_angle + a_ref1_offset + a_b_offset,
            pos_ref1=ref1_angle,
            pos_c=ref2_angle + c_ref2_offset,
            pos_d=ref2_angle + c_ref2_offset + c_d_offset,
            pos_ref2=ref2_angle,
            pos_tdc1=tdc_angle, pos_tdc2=tdc_angle,
            a_width=a_width, b_width=b_width,
            c_width=c_width, d_width=d_width,
            ref1_width=ref1_width, ref2_width=ref2_width,
            tdc1_width=tdc_width, tdc2_width=tdc_width,
        )
