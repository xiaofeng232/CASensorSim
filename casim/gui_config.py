"""X1+ CA Generator 实机 GUI 参数 → SensorSpec 转换。

实机 GUI 参数面板（"Offsets (not higher than 2500)"，单位 mDeg）：

    numOffTeeth              齿轮齿数（单一值，A/B 与 C/D 同齿轮）
                             tooth segment = 360000 / numOffTeeth [mDeg/齿]
    Tooth_Length_A..D        拾波 A/B/C/D 高电平宽度 [mDeg]
    Offset_A..D              拾波脉冲中心相对本组 REF 脉冲中心的偏移 [mDeg]
                             （A/B 相对 REF1, C/D 相对 REF2；实机约束 ≤ 2500）
    TDC1_RE / TDC1_FE        TDC1（组 1, CH00）上升/下降沿位置 [mDeg], TDC(0°) 基准
    TDC2_RE / TDC2_FE        TDC2（组 2, CH07）上升/下降沿位置 [mDeg], TDC 基准
    REF1_TDC_RE / _FE        REF1 上升/下降沿位置 [mDeg], TDC(0°) 基准
    REF2_BDC_RE / _FE        REF2 上升/下降沿位置 [mDeg], BDC(180°) 基准

约定（CA Generator Model 新模型，与 casim 中心-中心约定一致）：
    - 1° = 1000 mDeg, 1 转 = 360000 mDeg；
    - Offset = 拾波脉冲中心 − 本组 REF 脉冲中心（中心-中心偏移，ACM-20 估算约定）;
    - RE/FE 定义一个脉冲: 宽度 = (FE − RE) mod 360, 中心 = RE + 宽度/2;
    - REF1 与 TDC 以 TDC(0°) 为基准, REF2 以 BDC(180°) 为基准（换算后
      REF1/REF2 为同一 BDC 标记的冗余拾波）;
    - TDC1/TDC2 为两个独立冗余传感器, TDC1 → CH00, TDC2 → CH07,
      各自独立位置/宽度 (同读 1 个 TDC 标记)。
"""
from __future__ import annotations

from .sensor import SensorSpec

MD = 1e-3        # mDeg -> deg
BDC = 180.0      # BDC 在 TDC 基准下的位置 [deg]


def _pulse(re_md: float, fe_md: float, ref_deg: float = 0.0):
    """上升/下降沿位置 (mDeg, ref_deg 基准) → (中心, 宽度) [deg]。"""
    re = (ref_deg + re_md * MD) % 360.0
    width = ((fe_md - re_md) * MD) % 360.0
    center = (re + 0.5 * width) % 360.0
    return center, width


def gui_to_spec(d: dict) -> SensorSpec:
    """实机 GUI 参数字典 (mDeg) → SensorSpec (deg)。

    字典键为实机 GUI 参数名（见模块 docstring）；"meta" 等附加键忽略。
    """
    teeth = int(d["numOffTeeth"])
    if not 2 <= teeth <= 360:
        raise ValueError(f"numOffTeeth 超出范围: {teeth}")

    ref1_c, ref1_w = _pulse(d["REF1_TDC_RE"], d["REF1_TDC_FE"], 0.0)
    ref2_c, ref2_w = _pulse(d["REF2_BDC_RE"], d["REF2_BDC_FE"], BDC)
    tdc1_c, tdc1_w = _pulse(d["TDC1_RE"], d["TDC1_FE"], 0.0)
    tdc2_c, tdc2_w = _pulse(d["TDC2_RE"], d["TDC2_FE"], 0.0)

    return SensorSpec(
        teeth=teeth,
        tooth0=0.0, ref1_mark=0.0, ref2_mark=0.0, tdc_mark=0.0,
        pos_a=(ref1_c + d["Offset_A"] * MD) % 360.0,
        pos_b=(ref1_c + d["Offset_B"] * MD) % 360.0,
        pos_c=(ref2_c + d["Offset_C"] * MD) % 360.0,
        pos_d=(ref2_c + d["Offset_D"] * MD) % 360.0,
        pos_ref1=ref1_c, pos_ref2=ref2_c,
        pos_tdc1=tdc1_c, pos_tdc2=tdc2_c,
        a_width=d["Tooth_Length_A"] * MD,
        b_width=d["Tooth_Length_B"] * MD,
        c_width=d["Tooth_Length_C"] * MD,
        d_width=d["Tooth_Length_D"] * MD,
        ref1_width=ref1_w, ref2_width=ref2_w,
        tdc1_width=tdc1_w, tdc2_width=tdc2_w,
    )
