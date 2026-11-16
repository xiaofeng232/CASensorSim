"""飞轮传感器安装位置图（配置可视化，对应 Port CAS 规格的安装图）。

0° 在正上方 (TDC = 0°), 角度顺时针增加 (发动机前进方向)。
"""
from __future__ import annotations

import numpy as np

from .sensor import SensorSpec

_COLORS = {
    "A": "#1f77b4", "B": "#2ca02c", "REF1": "#d62728",
    "C": "#ff7f0e", "D": "#8c564b", "REF2": "#17becf",
    "TDC1": "#9467bd", "TDC2": "#e377c2",
}
_LABELS = {
    "A": "PU A1", "B": "PU B1", "REF1": "REF1",
    "C": "PU C1", "D": "PU D1", "REF2": "REF2",
    "TDC1": "TDC1", "TDC2": "TDC2",
}


def make_diagram(spec: SensorSpec, path: str,
                 title: str = "Sensor installation (configurable)"):
    """绘制飞轮: 齿轮齿、REF1/REF2/TDC 标记、各传感器安装位置。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    P = spec.tooth_period

    def xy(r, a_deg):
        a = np.deg2rad(a_deg)
        return r * np.sin(a), r * np.cos(a)

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_xlim(-1.8, 1.8)
    ax.set_ylim(-1.8, 1.8)

    # 圆盘 + 齿
    ax.add_patch(plt.Circle((0, 0), 0.95, color="0.87", ec="0.15",
                            lw=1.5, zorder=1))
    for k in range(spec.teeth):
        a = spec.tooth0 + k * P
        hw = 0.5 * P * 0.45
        pts = [xy(0.95, a - hw), xy(1.08, a - 0.55 * hw),
               xy(1.08, a + 0.55 * hw), xy(0.95, a + hw)]
        ax.add_patch(Polygon(pts, closed=True, color="0.87", ec="0.15",
                             lw=0.5, zorder=2))

    # 0°/180° 参考线
    for a in (0.0, 180.0):
        (x1, y1), (x2, y2) = xy(1.45, a), xy(-1.45, a)
        ax.plot([x1, x2], [y1, y2], ls=(0, (6, 4)), color="0.45",
                lw=0.8, zorder=0)
    ax.plot([-1.45, 1.45], [0, 0], ls=(0, (2, 3)), color="0.6", lw=0.6, zorder=0)
    ax.text(*xy(1.58, 0.0), "TDC = 0°", ha="center", va="center", fontsize=10)
    ax.text(*xy(1.58, 180.0), "BDC = 180°", ha="center", va="center",
            fontsize=10)

    # 飞轮标记 (REF2 先画, 避免与 REF1 重合时被盖住)
    def mark(a, color):
        (x1, y1), (x2, y2) = xy(0.86, a), xy(1.00, a)
        ax.plot([x1, x2], [y1, y2], color=color, lw=7,
                solid_capstyle="round", zorder=3)

    mark(spec.ref2_mark, _COLORS["REF2"])
    mark(spec.ref1_mark, _COLORS["REF1"])
    mark(spec.tdc_mark, _COLORS["TDC1"])   # 单一 TDC 标记, TDC1/TDC2 同读

    # 传感器 (齿顶外侧三角, 顶点朝飞轮)
    positions = (("A", spec.pos_a), ("B", spec.pos_b), ("REF1", spec.pos_ref1),
                 ("C", spec.pos_c), ("D", spec.pos_d), ("REF2", spec.pos_ref2),
                 ("TDC1", spec.pos_tdc1), ("TDC2", spec.pos_tdc2))
    for name, pos in positions:
        color = _COLORS[name]
        pts = [xy(1.16, pos), xy(1.30, pos - 3.5), xy(1.30, pos + 3.5)]
        ax.add_patch(Polygon(pts, closed=True, color=color, ec="k",
                             lw=0.4, zorder=4))
        ax.text(*xy(1.52, pos), _LABELS[name], ha="center", va="center",
                fontsize=9, color=color, fontweight="bold", zorder=5)

    ax.set_title(title, fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
