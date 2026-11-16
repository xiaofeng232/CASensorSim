"""预检：模拟 C 移植的运算顺序，验证与 casim 黄金模型位级一致。

C 计划采用:
  t[k]  = (double)k * dt                      (对应 np.arange 的 start + i*step, start=0)
  θ[k]  = start + cumsum(inc)[k-1] 的顺序累加  (cum += inc; theta = start + cum)
  inc[k]= ((0.5*(rpm[k]+rpm[k+1])) * 6.0) * dt
  mDeg 配置 /1000.0 转 deg
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from casim.generator import CASimulator
from casim.profile import ConstantSpeed, RampSpeed
from casim.sensor import SensorSpec
from casim.ssi import SSISpec

def h31(a):
    h = 0
    for v in a:
        h = (h * 31 + int(v)) & 0xFFFFFFFF
    return h

# ---- X1+ 实机参数 (mDeg -> deg, 与 C 的 /1000.0 相同) ----
spec = SensorSpec(
    teeth=186,
    tooth0=0/1000.0,
    ref1_mark=0/1000.0, ref2_mark=0/1000.0, tdc_mark=0/1000.0,
    pos_a=180000/1000.0, pos_b=180300/1000.0, pos_ref1=180000/1000.0,
    pos_c=180000/1000.0, pos_d=180300/1000.0, pos_ref2=180000/1000.0,
    pos_tdc1=200/1000.0, pos_tdc2=200/1000.0,
    a_width=600/1000.0, b_width=600/1000.0, c_width=600/1000.0, d_width=600/1000.0,
    ref1_width=1000/1000.0, ref2_width=1000/1000.0,
    tdc1_width=1000/1000.0, tdc2_width=1000/1000.0,
)
sim = CASimulator(ConstantSpeed(60.0), dt=20e-6, duration=5.0, sensors=spec)
res = sim.run()
N = res.N if hasattr(res, "N") else len(res.t)
print("N =", N, " dt =", res.dt)

# ---- 模拟 C 循环 ----
dt = 20e-6
t_c = np.arange(N) * dt
print("t identity:", bool(np.array_equal(t_c, res.t)))

rpm_arr = np.full(N, 60.0)
inc = ((0.5 * (rpm_arr[1:] + rpm_arr[:-1])) * 6.0) * dt
cum = 0.0
theta_c = np.empty(N)
start = 0.0
for k in range(N):
    if k > 0:
        cum += inc[k-1]
    theta_c[k] = start + cum
print("theta identity:", bool(np.array_equal(theta_c, res.angle)))

# 方波 (C 顺序: (theta - center) + 0.5*width) % P, rel < L
P = 360.0 / 186
centers = spec.centers()
def sq(angle, period, L, center):
    rel = (angle - center + 0.5 * L) % period
    return (rel < L).astype(np.int8)
w = spec
ok = True
for name, (period, width) in {
    "A": (P, w.a_width), "B": (P, w.b_width),
    "REF1": (360.0, w.ref1_width), "C": (P, w.c_width),
    "D": (P, w.d_width), "REF2": (360.0, w.ref2_width),
    "TDC1": (360.0, w.tdc1_width), "TDC2": (360.0, w.tdc2_width),
}.items():
    s = sq(theta_c, period, width, centers[name])
    same = bool(np.array_equal(s, res.signals[name]))
    ok &= same
    print(f"  {name}: identity={same} hash={h31(s)} ones={int(s.sum())}")
print("all dig identity:", ok)

# SSI (C 顺序: a=(theta+off)%360; rne((a/360.0)*4096.0) % 4096)
def rne(x):
    f = np.floor(x)
    d = x - f
    out = np.where(d > 0.5, f + 1.0, np.where(d < 0.5, f, np.where(f % 2 == 0, f, f + 1.0)))
    return out
a = (theta_c + 0.0) % 360.0
ssi_c = (rne((a / 360.0) * 4096.0) % 4096.0).astype(np.int32)
print("ssi1 identity:", bool(np.array_equal(ssi_c, res.ssi1)),
      " hash:", h31(ssi_c), " hash_casim:", h31(res.ssi1))

# 边界: 首个 TDC1 上升沿
tg = res.signals["TDC1"]
idx = int(np.argmax(tg[1:] != tg[:-1]))  # 第一个变化点
print("first TDC1 edge idx:", idx, "t=", res.t[idx], "deg=", res.angle[idx])

# 哈希基线 (与网页 25 万样本验证值对照)
for name in ["TDC1", "REF1", "A", "B", "REF2", "C", "D", "TDC2"]:
    print(f"golden {name}: hash={h31(res.signals[name])} ones={int(res.signals[name].sum())}")
print("golden ssi1:", h31(res.ssi1), " ssi2:", h31(res.ssi2))
