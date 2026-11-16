"""命令行入口:

    python -m casim --rpm 60 --duration 5 --teeth 100 --out sim.csv --plot sim.png
"""
from __future__ import annotations

import argparse
import dataclasses
import json

import numpy as np

from .generator import CASimulator
from .gui_config import gui_to_spec
from .profile import ConstantSpeed, RampSpeed, Fluctuation
from .sensor import SensorSpec
from .ssi import SSISpec


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        prog="casim", description="曲轴角度 (CA) 传感器信号仿真 - 转速驱动")
    p.add_argument("--rpm", type=float, default=60.0, help="转速 [rpm], 带符号 (默认 60)")
    p.add_argument("--start-rpm", type=float, default=None,
                   help="给定后进入斜坡模式: 从该转速按 --accel 加/减速到 --rpm")
    p.add_argument("--accel", type=float, default=5.0, help="斜坡模式加速度 [rpm/s]")
    p.add_argument("--duration", type=float, default=5.0, help="仿真时长 [s]")
    p.add_argument("--dt-us", type=float, default=50.0, help="采样周期 [us]")
    p.add_argument("--teeth", type=int, default=None,
                   help="飞轮齿轮齿数, A/B/C/D 共用 (默认 100, 可来自 --config)")
    p.add_argument("--config", default=None,
                   help="参数配置 JSON: 含 numOffTeeth 键 = 实机 GUI 格式 (mDeg, "
                        "见 configs/x1plus.json); 否则为 SensorSpec 字段名 (deg)")
    # ---- 传感器安装位置 (deg, 0°=飞轮0°, 顺时针为正) ----
    g = p.add_argument_group("安装位置配置 (0°=飞轮0°, 顺时针为正)")
    g.add_argument("--tooth0", type=float, default=None,
                   help="第 0 齿中心在飞轮上的位置 [deg]")
    g.add_argument("--ref1-mark", type=float, default=None,
                   help="REF1 标记在飞轮上的位置 [deg]")
    g.add_argument("--ref2-mark", type=float, default=None,
                   help="REF2 标记在飞轮上的位置 [deg]")
    g.add_argument("--tdc-mark", type=float, default=None,
                   help="TDC 标记在飞轮上的位置 [deg]")
    g.add_argument("--pos-a", type=float, default=None, help="PU A1 传感器安装角 [deg]")
    g.add_argument("--pos-b", type=float, default=None, help="PU B1 传感器安装角 [deg]")
    g.add_argument("--pos-ref1", type=float, default=None, help="REF1 传感器安装角 [deg]")
    g.add_argument("--pos-c", type=float, default=None, help="PU C1 传感器安装角 [deg]")
    g.add_argument("--pos-d", type=float, default=None, help="PU D1 传感器安装角 [deg]")
    g.add_argument("--pos-ref2", type=float, default=None, help="REF2 传感器安装角 [deg]")
    g.add_argument("--pos-tdc1", type=float, default=None,
                   help="TDC1 传感器安装角 [deg] (通道组 1, CH00)")
    g.add_argument("--pos-tdc2", type=float, default=None,
                   help="TDC2 传感器安装角 [deg] (通道组 2, CH07, 与 TDC1 冗余)")
    p.add_argument("--diagram", default=None, help="输出飞轮安装位置图 PNG")
    p.add_argument("--start-angle", type=float, default=0.0, help="初始曲轴角 [deg]")
    p.add_argument("--fluct-amp", type=float, default=0.0, help="转速波动幅值 [rpm]")
    p.add_argument("--fluct-freq", type=float, default=1.0, help="转速波动频率 [Hz]")
    p.add_argument("--ssi-offset", type=float, default=0.0,
                   help="SSI2 相对 SSI1 的角度偏移 [deg] (故障仿真)")
    p.add_argument("--pfb", type=int, default=0, choices=[0, 1, 2],
                   help="置位 PFB 故障位: 1=SSI1, 2=SSI2")
    p.add_argument("--out", default="casim.csv", help="输出 CSV 路径")
    p.add_argument("--plot", default=None, help="输出 PNG 图路径")
    return p.parse_args(argv)


def build_profile(args):
    if args.start_rpm is not None:
        prof = RampSpeed(start_rpm=args.start_rpm, target_rpm=args.rpm,
                         accel=args.accel)
    else:
        prof = ConstantSpeed(args.rpm)
    if args.fluct_amp:
        prof = Fluctuation(prof, args.fluct_amp, args.fluct_freq)
    return prof


def make_plot(res, path: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t, rpm = res.t, res.rpm

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    ax = axes[0]
    ax.plot(t, rpm, lw=1.2, color="tab:blue", label="speed [rpm]")
    ax.set_ylabel("rpm")
    ax.legend(loc="upper left")
    ax.grid(alpha=0.3)

    ax = axes[1]
    ax.plot(t, np.mod(res.angle, 360.0), lw=0.6,
            color="tab:orange", label="crank angle [deg]")
    ax.set_ylabel("deg")
    ax.set_ylim(-10, 370)
    ax.legend(loc="upper left")
    ax.grid(alpha=0.3)

    # 放大最后 1 转, 按 NI 9474 接线 (2 组 x 4 路) 显示
    last_rev = t[-1] - 360.0 / max(abs(rpm[-1]), 1e-9)
    m = t >= last_rev
    ax = axes[2]
    chans = res.channels()
    yoff = 0
    for i, ch in enumerate(chans):
        y = chans[ch][m]
        ax.step(t[m], y + yoff, where="post", lw=0.9)
        if ch in ("CH00", "CH04"):  # 两组边界
            ax.axhline(yoff - 0.5, color="0.4", lw=0.8)
        ax.annotate(ch, (t[m][0], yoff - 0.35), fontsize=7)
        yoff += 1
    ax.annotate("group 1 (A/B)", (t[m][0], 3.3), fontsize=7, color="0.3")
    ax.annotate("group 2 (C/D)", (t[m][0], 7.3), fontsize=7, color="0.3")
    ax.set_yticks(np.arange(yoff) + 0.5, list(chans), fontsize=8)
    ax.set_yticks(np.arange(yoff) + 0.0, minor=True)
    ax.set_ylabel("9474\nchannels")
    ax.grid(alpha=0.3, which="both")
    ax.set_xlabel("time [s]")

    fig.suptitle("CA sensor simulation - speed driven (virtual flywheel)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def build_spec(args):
    """优先级: 默认值 < --config < 命令行显式参数。"""
    if args.config:
        with open(args.config, encoding="utf-8") as f:
            cfg = json.load(f)
        if "numOffTeeth" in cfg:
            spec = gui_to_spec(cfg)
        else:
            kw = {k: v for k, v in cfg.items()
                  if k in SensorSpec.__dataclass_fields__}
            spec = SensorSpec(**kw)
    else:
        spec = SensorSpec()
    ov = {}
    for field in ("tooth0", "ref1_mark", "ref2_mark", "tdc_mark",
                  "pos_a", "pos_b", "pos_ref1", "pos_c", "pos_d",
                  "pos_ref2", "pos_tdc1", "pos_tdc2"):
        v = getattr(args, field)
        if v is not None:
            ov[field] = v
    if args.teeth is not None:
        ov["teeth"] = args.teeth
    if ov:
        spec = dataclasses.replace(spec, **ov)
    return spec


def main(argv=None):
    args = parse_args(argv)
    sensors = build_spec(args)
    ssi = SSISpec(ssi2_offset=args.ssi_offset,
                  pfb1=(args.pfb == 1), pfb2=(args.pfb == 2))
    sim = CASimulator(build_profile(args), dt=args.dt_us * 1e-6,
                      duration=args.duration, start_angle=args.start_angle,
                      sensors=sensors, ssi=ssi)
    res = sim.run()
    res.to_csv(args.out)
    rev = (res.angle[-1] - res.angle[0]) / 360.0
    print(f"wrote {args.out}: {res.t.size} samples, "
          f"{res.angle[-1] - res.angle[0]:.1f} deg ({rev:.2f} rev)")
    if args.plot:
        make_plot(res, args.plot)
        print(f"wrote {args.plot}")
    if args.diagram:
        from .diagram import make_diagram
        make_diagram(sensors, args.diagram)
        print(f"wrote {args.diagram}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
