"""CASim - 曲轴转角 (Crank Angle, CA) 传感器信号仿真器。

根据发动机转速（虚拟飞轮）生成 WinGD X 系列主机所用的曲轴角度
传感器信号:

- 拾波信号 A / B / C / D (飞轮齿轮齿, 两对冗余)
- 参考脉冲 REF1 / REF2 (每转 1 次)
- 上止点 TDC / OTP (第 1 缸, 每转 1 次)
- SSI 绝对角度字 (12 bit 角度 + PFB 故障位, 双通道冗余)

信号模型实现自 WinGD《CA Generator Model》文档中的正弦方波公式。
"""
from .profile import SpeedProfile, ConstantSpeed, RampSpeed, Fluctuation, CurveSpeed
from .sensor import SensorSpec, square_wave, square_wave_sine
from .ssi import SSISpec, PFB_SHIFT
from .generator import CASimulator, SimResult
from .diagram import make_diagram
from .gui_config import gui_to_spec

__all__ = [
    "SpeedProfile", "ConstantSpeed", "RampSpeed", "Fluctuation", "CurveSpeed",
    "SensorSpec", "square_wave", "square_wave_sine",
    "SSISpec", "PFB_SHIFT",
    "CASimulator", "SimResult",
    "make_diagram",
    "gui_to_spec",
]
