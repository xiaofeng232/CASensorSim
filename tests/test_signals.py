"""CA 传感器信号仿真 - 验证测试。"""
import json
import os

import numpy as np
import pytest

from casim.generator import CASimulator
from casim.gui_config import gui_to_spec
from casim.profile import ConstantSpeed, RampSpeed
from casim.sensor import SensorSpec, square_wave, square_wave_sine
from casim.ssi import SSISpec, PFB_SHIFT


def make_sim(rpm=60.0, dt=10e-6, duration=2.0, teeth=100, **spec_kw):
    spec = SensorSpec(teeth=teeth, **spec_kw)
    sim = CASimulator(ConstantSpeed(rpm), dt=dt, duration=duration,
                      sensors=spec)
    return sim, spec


def rise_edges(sig, ang):
    """上升沿对应的曲轴角 (deg)。"""
    i = np.where((sig[1:] == 1) & (sig[:-1] == 0))[0] + 1
    return ang[i]


def count_rising(sig):
    return int(np.sum((sig[1:] == 1) & (sig[:-1] == 0)))


def test_tooth_frequency():
    sim, _ = make_sim(rpm=60.0, duration=2.0)
    res = sim.run()
    # 60 rpm -> 1 转/s -> 100 齿/s -> 2 s 内 200 个 A 齿脉冲
    assert count_rising(res.signals["A"]) == pytest.approx(200, abs=2)


def test_ref_and_tdc_one_pulse_per_rev():
    sim, _ = make_sim(rpm=60.0, duration=3.0)
    res = sim.run()
    assert count_rising(res.signals["REF1"]) == 3
    assert count_rising(res.signals["REF2"]) == 3
    assert count_rising(res.signals["TDC1"]) == 3
    assert count_rising(res.signals["TDC2"]) == 3


def test_pulse_width_in_degrees():
    sim, _ = make_sim(rpm=60.0, dt=2e-6, duration=1.0)
    res = sim.run()
    # 60 rpm -> 360 deg/s
    rate = res.rpm[0] * 6.0  # deg/s
    a = res.signals["A"]
    first = np.where(a == 1)[0][0]
    end = np.where(a[first:] == 0)[0][0] + first
    width_deg = (end - first) * res.dt * rate
    assert width_deg == pytest.approx(1.8, abs=0.05)  # 默认 a_width


def test_a_b_center_offset_half_pitch():
    sim, _ = make_sim(rpm=60.0, dt=2e-6, duration=1.0)
    res = sim.run()
    a_c = rise_edges(res.signals["A"], res.angle) + 0.9   # 宽度 1.8 -> 半宽 0.9
    b_c = rise_edges(res.signals["B"], res.angle) + 0.9
    delta = (b_c[0] - a_c[0]) % 3.6                        # 齿周期 360/100
    assert delta == pytest.approx(1.8, abs=0.05)           # 默认 a_b_offset


def test_a_center_vs_ref1():
    sim, spec = make_sim(rpm=60.0, dt=2e-6, duration=1.0)
    res = sim.run()
    # REF1 中心在 0 deg, A 中心应滞后 a_ref1_offset = 0.9 deg (模齿周期)
    a_c = np.mod(rise_edges(res.signals["A"], res.angle) + 0.9,
                 spec.tooth_period)
    assert a_c[0] == pytest.approx(0.9, abs=0.05)


def test_sine_model_matches_square_wave():
    th = np.linspace(0.0, 723.7, 20000)
    x1 = square_wave(th, 3.6, 1.8, 0.9)
    x2 = square_wave_sine(th, 3.6, 1.8, 0.9)
    assert np.sum(x1 != x2) <= 2  # 仅边界采样点可能差异


def test_ssi_word_tracks_angle():
    sim, _ = make_sim(rpm=60.0, dt=20e-6, duration=1.0)
    res = sim.run()
    counts = (res.angle % 360.0) / 360.0 * 4096.0
    word = (res.ssi1.astype(np.int64) & 0xFFF).astype(np.float64)
    # 圆域差值 (角度跨 360° 时 SSI 字会回卷)
    dc = np.abs(word - counts)
    err_counts = np.minimum(dc, 4096.0 - dc)
    assert np.max(err_counts * 360.0 / 4096.0) < 0.05  # 1/4096 转 = 0.088 deg


def test_ssi_offset_and_pfb():
    ssi = SSISpec(ssi2_offset=0.5, pfb2=True)
    sim = CASimulator(ConstantSpeed(60.0), dt=20e-6, duration=0.5,
                      sensors=SensorSpec(), ssi=ssi)
    res = sim.run()
    assert np.all(((res.ssi2 >> PFB_SHIFT) & 1) == 1)
    c1 = res.ssi1.astype(np.int64) & 0xFFF
    c2 = res.ssi2.astype(np.int64) & 0xFFF
    delta = np.abs((c1 - c2 + 2048) % 4096 - 2048).astype(np.float64)
    assert np.all(np.abs(delta - 0.5 * 4096 / 360.0) < 2.0)  # ~5.7 counts


def test_ramp_angle_integral():
    # 0 -> 60 rpm @ 10 rpm/s, 6 s: 转角 = 0.5*60*6 [rev*s] * 6 [deg/rev*s] = 1080 deg
    prof = RampSpeed(start_rpm=0.0, target_rpm=60.0, accel=10.0)
    sim = CASimulator(prof, dt=20e-6, duration=6.0)
    res = sim.run()
    assert res.angle[-1] == pytest.approx(1080.0, abs=0.5)
    assert res.rpm[-1] == pytest.approx(60.0, abs=0.5)


def test_installation_position_model():
    # center(齿) = (pos - tooth0) mod 齿周期; center(标记) = (pos - mark) mod 360
    spec = SensorSpec(teeth=100, tooth0=5.0,
                      ref1_mark=20.0, pos_ref1=35.0,
                      ref2_mark=80.0, pos_ref2=85.0,
                      tdc_mark=135.0, pos_tdc1=135.0, pos_tdc2=135.0,
                      pos_a=30.0, pos_b=31.5, pos_c=60.0, pos_d=61.5)
    P = 360.0 / 100
    c = spec.centers()
    assert c["A"] == pytest.approx((30.0 - 5.0) % P)
    assert c["B"] == pytest.approx((31.5 - 5.0) % P)
    assert c["REF1"] == pytest.approx(15.0)
    assert c["REF2"] == pytest.approx(5.0)
    # 标记与传感器同角安装 => 脉冲中心落在飞轮 0° (TDC1/TDC2 同读 1 标记)
    assert c["TDC1"] == pytest.approx(0.0)
    assert c["TDC2"] == pytest.approx(0.0)


def test_signal_center_follows_installation():
    # 标记在飞轮上向前移 5° => 脉冲中心后移 5°
    s1 = SensorSpec(teeth=100, ref1_mark=0.0, pos_ref1=0.0)
    s2 = SensorSpec(teeth=100, ref1_mark=5.0, pos_ref1=0.0)
    assert s1.centers()["REF1"] == pytest.approx(0.0)
    assert s2.centers()["REF1"] == pytest.approx(355.0)
    # 实际信号验证
    sim = CASimulator(ConstantSpeed(60.0), dt=2e-6, duration=1.0, sensors=s2)
    res = sim.run()
    rise = rise_edges(res.signals["REF1"], res.angle)
    center = (rise[0] + s2.ref1_width / 2) % 360.0
    assert center == pytest.approx(355.0, abs=0.05)


def test_from_offsets_equivalence():
    spec = SensorSpec.from_offsets(teeth=100, a_ref1_offset=0.9,
                                   a_b_offset=1.8, ref1_angle=10.0)
    c = spec.centers()
    assert c["REF1"] == pytest.approx(10.0)
    assert c["A"] == pytest.approx((10.0 + 0.9) % 3.6)
    assert c["B"] == pytest.approx((10.0 + 0.9 + 1.8) % 3.6)


def test_single_flywheel_teeth():
    # 单飞轮单齿轮: C/D 与 A/B 共用同一齿数与齿位网格 (100 齿, 周期 3.6°),
    # 仅安装位置 pos_c/pos_d 独立
    spec = SensorSpec(teeth=100, pos_c=25.0, pos_d=27.5)
    c = spec.centers()
    assert c["A"] == pytest.approx(0.9)          # 默认 pos_a, 100 齿周期
    assert c["C"] == pytest.approx(25.0 % 3.6)
    assert c["D"] == pytest.approx(27.5 % 3.6)
    sim = CASimulator(ConstantSpeed(60.0), dt=2e-6, duration=1.0, sensors=spec)
    res = sim.run()
    # 60 rpm -> A 与 C 同读一个齿轮, 齿频均 = 1 * 100 = 100 脉冲/s
    assert count_rising(res.signals["C"]) == pytest.approx(100, abs=2)
    assert count_rising(res.signals["A"]) == pytest.approx(100, abs=2)


def test_ni9474_wiring_channels():
    from casim.generator import WIRING
    sim, _ = make_sim(rpm=60.0, dt=20e-6, duration=1.0)
    res = sim.run()
    ch = res.channels()
    # 通道顺序 = 接线表
    assert list(ch) == [c for c, _ in WIRING]
    # TDC1→CH00, TDC2→CH07 (两个独立冗余传感器; 默认配置两者同值)
    assert np.array_equal(ch["CH00"], res.signals["TDC1"])
    assert np.array_equal(ch["CH07"], res.signals["TDC2"])
    assert np.array_equal(ch["CH00"], ch["CH07"])
    assert np.array_equal(ch["CH01"], res.signals["REF1"])
    assert np.array_equal(ch["CH02"], res.signals["A"])
    assert np.array_equal(ch["CH03"], res.signals["B"])
    assert np.array_equal(ch["CH04"], res.signals["REF2"])
    assert np.array_equal(ch["CH05"], res.signals["C"])
    assert np.array_equal(ch["CH06"], res.signals["D"])
    # CSV 列 = t/angle/rpm + CH00..CH07 + ssi1/ssi2
    assert res.csv_columns() == (["t_s", "angle_deg", "speed_rpm"]
                                 + [c for c, _ in WIRING] + ["ssi1", "ssi2"])


def test_direction_negative_rpm():
    # 反转: 角度应减小, A 齿序列频率不变
    sim, _ = make_sim(rpm=-60.0, duration=1.0)
    res = sim.run()
    assert res.angle[-1] < res.angle[0]
    assert count_rising(res.signals["A"]) == pytest.approx(100, abs=2)


# ---- X1+ 实机 GUI 配置 (configs/x1plus.json) ----

def load_x1plus():
    with open(os.path.join(os.path.dirname(__file__), os.pardir,
                           "configs", "x1plus.json"), encoding="utf-8") as f:
        return json.load(f)


def _edge_md(sig, ang):
    """上升/下降沿位置 [mDeg, mod 360000]。"""
    ir = np.where((sig[1:] == 1) & (sig[:-1] == 0))[0] + 1
    ifa = np.where((sig[1:] == 0) & (sig[:-1] == 1))[0] + 1
    return (np.mod(ang[ir] * 1e3, 360000.0),
            np.mod(ang[ifa] * 1e3, 360000.0))


def _cdist(a, b, mod):
    d = np.mod(a - b, mod)
    return np.minimum(d, mod - d)


def test_x1plus_gui_config_conversion():
    # 实机 GUI 值: 186 齿, REF1/REF2 同处 BDC(180°), TDC RE 359.7°/FE 0.7°,
    # A/B 中心 = REF1 中心 + 0/0.3°, 齿脉冲宽 0.6°
    spec = gui_to_spec(load_x1plus())
    assert spec.teeth == 186
    # tooth segment = 360000/186 = 1935.4 mDeg (GUI 显示值)
    assert spec.tooth_period * 1000.0 == pytest.approx(1935.4, abs=0.1)
    P = spec.tooth_period
    c = spec.centers()
    assert c["REF1"] == pytest.approx(180.0)
    assert c["REF2"] == pytest.approx(180.0)   # BDC 基准 359.5° == TDC 基准 179.5°
    assert c["TDC1"] == pytest.approx(0.2)
    assert c["TDC2"] == pytest.approx(0.2)     # X1+ 两路 TDC 配置同值
    assert min(c["A"] % P, abs(c["A"] % P - P)) < 1e-6   # 180° = 93 整齿周期
    assert c["B"] % P == pytest.approx(0.3, abs=1e-6)
    assert c["C"] == c["A"]
    assert c["D"] == c["B"]
    assert spec.ref1_width == pytest.approx(1.0)
    assert spec.ref2_width == pytest.approx(1.0)
    assert spec.tdc1_width == pytest.approx(1.0)
    assert spec.tdc2_width == pytest.approx(1.0)
    assert spec.a_width == spec.b_width == spec.c_width == spec.d_width
    assert spec.a_width == pytest.approx(0.6)


def test_x1plus_machine_edges():
    # 全通道边沿与实机 GUI 参数一致 (mDeg, dt=2us -> 0.72 mDeg/点)
    spec = gui_to_spec(load_x1plus())
    sim = CASimulator(ConstantSpeed(60.0), dt=2e-6, duration=3.0, sensors=spec)
    res = sim.run()
    P_md = 360000.0 / 186.0
    # A: 每转 186 上升沿, 全部落在齿网格 (RE 179.7° + k*segment) 上
    rs, fs = _edge_md(res.signals["A"], res.angle)
    assert len(rs) == pytest.approx(558, abs=3)
    assert np.max(_cdist(rs, 179700.0, P_md)) < 1.5
    # B: 网格 RE 180.0° + k*segment (中心偏移 0.3°)
    rs, _ = _edge_md(res.signals["B"], res.angle)
    assert np.max(_cdist(rs, 180000.0, P_md)) < 1.5
    # C/D 与 A/B 冗余一致
    assert np.array_equal(res.signals["C"], res.signals["A"])
    assert np.array_equal(res.signals["D"], res.signals["B"])
    # REF1: 每转 1 次, RE 179.5° / FE 180.5°
    rs, fs = _edge_md(res.signals["REF1"], res.angle)
    assert len(rs) == 3
    assert np.max(_cdist(rs, 179500.0, 360000.0)) < 1.5
    assert np.max(_cdist(fs, 180500.0, 360000.0)) < 1.5
    # REF2 (BDC 基准 359.5°/0.5°) 与 REF1 同标记
    assert np.array_equal(res.signals["REF2"], res.signals["REF1"])
    # TDC1/TDC2: RE 359.7° / FE 0.7° (X1+ 两路配置同值 => TDC1≡TDC2, CH00≡CH07)
    rs, fs = _edge_md(res.signals["TDC1"], res.angle)
    assert len(rs) == 3
    assert np.max(_cdist(rs, 359700.0, 360000.0)) < 1.5
    assert np.max(_cdist(fs, 700.0, 360000.0)) < 1.5
    rs2, fs2 = _edge_md(res.signals["TDC2"], res.angle)
    assert np.max(_cdist(rs2, 359700.0, 360000.0)) < 1.5
    assert np.max(_cdist(fs2, 700.0, 360000.0)) < 1.5
    assert np.array_equal(res.channels()["CH00"], res.channels()["CH07"])


def test_cli_config_gui_format(tmp_path):
    import casim.__main__ as cli
    cfg = os.path.join(os.path.dirname(__file__), os.pardir,
                       "configs", "x1plus.json")
    out = tmp_path / "x1plus.csv"
    rc = cli.main(["--config", cfg, "--rpm", "60", "--duration", "1.0",
                   "--dt-us", "20", "--out", str(out)])
    assert rc == 0
    d = np.genfromtxt(out, delimiter=",", names=True)
    # 186 齿 -> 1 s (60 rpm = 1 转) A 上升沿 ≈ 186
    a = d["CH02"].astype(int)
    assert np.sum((a[1:] == 1) & (a[:-1] == 0)) == pytest.approx(186, abs=3)


def test_cli_config_spec_format(tmp_path):
    import casim.__main__ as cli
    cfg = tmp_path / "spec.json"
    cfg.write_text('{"teeth": 99, "pos_a": 5.0, "a_width": 0.4}',
                   encoding="utf-8")
    out = tmp_path / "s.csv"
    rc = cli.main(["--config", str(cfg), "--rpm", "60", "--duration", "0.5",
                   "--dt-us", "20", "--out", str(out)])
    assert rc == 0
    d = np.genfromtxt(out, delimiter=",", names=True)
    a = d["CH02"].astype(int)
    # 99 齿, 0.5 s = 180 deg -> 180/(360/99) ≈ 49.5 个上升沿
    assert np.sum((a[1:] == 1) & (a[:-1] == 0)) == pytest.approx(49.5, abs=2)
