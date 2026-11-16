"""SSI (Synchronous Serial Interface) 绝对角度字编码。

WinGD 配置（WSG 需求规格 / 便携式 CAS 规格）：
- 12 bit 角度分辨率 (4096 步/转), 单圈绝对值
- 每帧 13 bit = 12 bit 角度 (MSB first) + 1 位 PFB (Power Fail Bit)
- 双通道冗余 (SSI1 / SSI2), RS-485 差分, 默认 500 kHz 时钟
- 支持 double readout (环形寄存器双读): 时钟不中断时同一角度再发一遍,
  主机用两次读值对比校验完整性
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

PFB_SHIFT = 12  # 12 bit 角度 + bit12 = PFB


@dataclass
class SSISpec:
    resolution: int = 4096     # 12 bit 单圈
    ssi1_offset: float = 0.0   # deg, SSI1 角度偏移 (故障仿真)
    ssi2_offset: float = 0.0   # deg, SSI2 角度偏移 (故障仿真)
    pfb1: bool = False         # SSI1 故障位 (Power Fail Bit)
    pfb2: bool = False         # SSI2 故障位
    double_readout: bool = True  # 环形寄存器双读 (协议层, 仿真中体现为两次读值相同)

    def _counts(self, angle, offset: float) -> np.ndarray:
        a = (np.asarray(angle, dtype=float) + offset) % 360.0
        return (np.round(a / 360.0 * self.resolution) % self.resolution).astype(np.int32)

    def words(self, angle):
        """每个采样时刻的 SSI 读字 (16 bit 容器, bit12 = PFB)。"""
        w1 = self._counts(angle, self.ssi1_offset)
        w2 = self._counts(angle, self.ssi2_offset)
        if self.pfb1:
            w1 = w1 | np.int32(1 << PFB_SHIFT)
        if self.pfb2:
            w2 = w2 | np.int32(1 << PFB_SHIFT)
        return w1, w2

    @staticmethod
    def decode(word: int):
        """返回 (角度计数, PFB)。"""
        w = int(word)
        return w & ((1 << PFB_SHIFT) - 1), bool(w >> PFB_SHIFT)
