# CASensorSim — 由转速仿真曲轴角度 (CA) 传感器信号

仿真 WinGD X 系列主机（WiCE ECS）识别曲轴转角所用的传感器信号。
输入发动机转速曲线，输出与真机一致的传感器时序：

| 信号 | 含义 | 频率 |
|------|------|------|
| A, B | 齿轮拾波（对 1），A/B 错半齿距判转向 | `rpm × 齿数 / 60` |
| C, D | 齿轮拾波（对 2，冗余通道） | 同上 |
| REF1 / REF2 | 飞轮参考标记脉冲 | 每转 1 次 |
| TDC1 / TDC2 | 第 1 缸上止点 (OTP)，两个冗余传感器 | 每转 1 次 |
| SSI1 / SSI2 | 绝对角度字：12 bit 角度 + PFB 故障位 | 每采样点 |

## Zephyr 固件 app

本仓库根目录同时是 Zephyr 固件 app（NUCLEO-F429ZI，试验台硬件为同型 Nucleo
板，见 KiCad 工程）。在 zephyrproject 工作区内已注册为 west 项目
`apps/cas-sim`：

```sh
west build -b nucleo_f429zi apps/cas-sim
west flash -d apps/cas-sim/build
```

固件移植状态与位级一致要求见 `HANDOFF.md` §7（活动任务：`cas_core` /
`cas_fault` 移植，故障注入 FR-30~FR-34）。

## 输出通道（按 NI 9474 接线，2 组 × 4 路）

| 通道 | 组 1 | 组 2 |
|------|------|------|
| CH00 / CH04 | TDC1 | REF2 |
| CH01 / CH05 | REF1 | C |
| CH02 / CH06 | A | D |
| CH03 / CH07 | B | TDC2 |

CSV 列顺序即 `t_s, angle_deg, speed_rpm, CH00..CH07, ssi1, ssi2`。

## 原理

1. **虚拟飞轮**：对转速梯形积分得曲轴角 `θ(t)`，
   `dθ[deg] = rpm × 6 [deg/s] × dt`（支持常数 / 斜坡加速 / 往复循环 / 正弦波动 / 任意折点曲线）。
2. **方波信号模型**（实现自 WinGD《CA Generator Model》公式）：
   每个传感器是周期 `T`、宽度 `L`、中心位于偏移角 `O` 的方波：

   ```
   y(θ) = sin(b·θ + c) + d > 0
   b = 2π/T
   c = O - REF + T/4          （脉冲中心对齐, ACM-20 中心-中心偏移约定）
   d = -cos(b·L/2)            （高电平宽度 = L, 与文档 Eq.29 联立的闭式解）
   ```

   对正弦取阈值后输出即理想方波；包内同时提供正弦参数化
   （`square_wave_sine`）与直接方波（`square_wave`）两种实现，
   数值等价，正弦版可直接对照 LabVIEW 移植。
3. **SSI 字**：`word = round(θ/360 × 4096) & 0xFFF`，bit12 = PFB。
   双通道独立偏移，可仿真 SSI1/SSI2 偏差与故障位。

## 用法

```bash
pip install numpy matplotlib pytest

# 常量 60 rpm, 5 s, 100 齿 -> CSV + PNG
python -m casim --rpm 60 --duration 5 --teeth 100 --out out/sim.csv --plot out/sim.png

# 0 -> 120 rpm 斜坡 (10 rpm/s) + 转速波动
python -m casim --start-rpm 0 --rpm 120 --accel 10 --fluct-amp 2 --fluct-freq 3

# 故障仿真: SSI2 偏 0.5 deg 且置 PFB
python -m casim --rpm 60 --ssi-offset 0.5 --pfb 2

# X1+ 实机参数 (GUI Offsets 面板值, 186 齿)
python -m casim --config configs/x1plus.json --rpm 60 --duration 5 \
  --out out/x1plus.csv --plot out/x1plus.png --diagram out/x1plus_layout.png

# 测试
python -m pytest tests -q
```

**Web 交互仿真台**：浏览器直接打开 `web/ca_dashboard.html`（纯前端单文件，无需
安装/服务器）。左侧 = 全部参数（齿轮/标记/8 传感器安装角/脉冲宽度/转速曲线
常数·斜坡·波动/SSI·故障，角度参数一律 mDeg 0–360000）+ 虚拟飞轮传感器安装图 + 派生参数；右侧 = CH00–CH07
波形、CA 信号（θ/转速/SSI）、实时数据表与 CSV 导出（casim 列格式）。
公式与 `casim` 黄金模型逐项一致（含 `np.round` 银行家舍入），已做 25 万样本
逐位校验。
动态运行：▶ 运行 / ⏸ 暂停 / ⏹ 停止 + 时标（1×/0.25×/0.05× 慢放）/ 循环。
运行中每帧按当前参数逐 dt 实时推进（1 s 滚动示波），**可随时设定转速**
（常数 rpm / 斜坡 start→target+accel / 波动 基线±幅值·频率，修改即时生效，
不重算整条曲线）；停止后按当前参数回到静态预计算预览与 CSV 数据。
**故障注入**（FR-30/31/34）：通道断线（OFF = 高阻，波形画虚线灰平线 +「高阻 (OFF)」）、
**两组 CA 差值**（组 2 CH04–CH07 整体平移 Δ mDeg，使两组拾波各自估算的 CA 相差 Δ）、
SSI PFB 故障位 / 角度恒定 / 总失效（高阻，覆盖恒定）；默认全关、显式使能，
开关在运行中即时生效；全部关闭时输出与黄金模型位级一致（CSV 导出会标注
「含故障注入，高阻=0」）。
CA 角度 θ 按 **0–360000 mDeg**（1/1000 度）每转回绕显示（与 ECU 角度计数一致）；
波形画布随卡片宽度铺满，窗口缩放自适应。

Python API：

```python
from casim import CASimulator, ConstantSpeed, RampSpeed, SensorSpec, SSISpec

sim = CASimulator(RampSpeed(0, 60, accel=10), dt=50e-6, duration=8.0,
                  sensors=SensorSpec(teeth=100),
                  ssi=SSISpec(ssi2_offset=0.5, pfb2=True))
res = sim.run()
res.to_csv("out.csv")          # t, angle, rpm, A..TDC2, ssi1, ssi2
res.angle, res.signals["A"]    # np 数组
```

## 参数（SensorSpec，单位 deg CA）

角度基准：`0° = 飞轮 0°（TDC = 0° 参考）`，顺时针为正（发动机前进方向）。

**安装位置配置**（与 Port CAS 规格一致：齿数全局、宽度/位置按传感器单独配置、
虚拟飞轮计算角度）：

| 类别 | 参数 | 含义 |
|------|------|------|
| 飞轮齿轮 | `teeth` (默认 100) | A/B/C/D 齿数（**单飞轮单齿轮**，4 路拾波共用），齿周期 `360/teeth` |
| | `tooth0` | 第 0 齿中心在飞轮上的位置 |
| 飞轮标记 | `ref1_mark / ref2_mark / tdc_mark` | REF1/REF2/TDC 标记加工在飞轮上的位置（TDC 为唯一曲轴角，仅 1 个标记） |
| 传感器安装 | `pos_a / pos_b / pos_ref1 / pos_c / pos_d / pos_ref2 / pos_tdc1 / pos_tdc2` | 各传感器在发动机坐标系中的固定安装角（TDC1/TDC2 为两个冗余传感器） |
| 脉冲宽度 | `a_width … tdc2_width` | 各脉冲宽度（默认齿脉冲 1.8，REF 0.6，TDC 0.5） |

**脉冲中心由安装位置推导**（齿/标记转到传感器下方时产生脉冲）：

```
齿拾波 (A/B/C/D):  center = (pos - tooth0) mod (360/teeth)
标记   (REF1/2, TDC): center = (pos - mark)  mod 360
A-B 偏移 = pos_b - pos_a (默认半齿距 => 判转向)
```

`SensorSpec.from_offsets(...)` 提供旧式“脉冲中心偏移”构造方式
（`a_ref1_offset`、`a_b_offset`、`ref1_angle`…），与安装位置模型等效。

**安装位置可视化**：`python -m casim … --diagram out/layout.png`
绘制飞轮（齿、REF1/REF2/TDC 标记、8 个传感器安装角 A/B/REF1/C/D/REF2/TDC1/TDC2）。

示例（安装位置按实物布置）：

```bash
python -m casim --rpm 60 --teeth 100 \
  --ref1-mark 25 --pos-a 26 --pos-b 27.8 --pos-ref1 28 \
  --ref2-mark 40 --pos-c 45 --pos-d 46.8 --pos-ref2 43.5 \
  --tdc-mark 135 --pos-tdc 135 \
  --out out/sim.csv --plot out/sim.png --diagram out/layout.png
```

## X1+ 实机参数配置（--config）

`--config configs/x1plus.json` 直接加载 X1+ 试验台 CA Generator GUI
"Offsets" 面板的实机值（`casim/gui_config.py` 负责换算）。两种 JSON 格式：

- **GUI 格式**（含 `numOffTeeth` 键，单位 mDeg）：GUI 面板字段原样照抄；
- **SensorSpec 格式**：包内字段名（单位 deg），见上表。

优先级：默认值 < `--config` < 命令行显式参数（如 `--teeth`）。

**GUI 字段 → 模型映射**（1° = 1000 mDeg，1 转 = 360000 mDeg）：

| GUI 字段 (mDeg) | 含义 | 换算 |
|------|------|------|
| `numOffTeeth` | 齿轮齿数（单飞轮单齿轮，A/B/C/D 共用） | `teeth = 186`，tooth segment = 360000/186 ≈ 1935.4 mDeg |
| `Tooth_Length_A..D` | 拾波高电平宽度 | `*_width = 值/1000`（本台 600 → 0.6°） |
| `Offset_A..D` | 拾波脉冲**中心**相对本组 REF 脉冲**中心**的偏移（A/B→REF1, C/D→REF2；实机约束 ≤ 2500） | `pos = REF 中心 + Offset/1000`（中心-中心约定，同 ACM-20） |
| `TDC1_RE / _FE` | TDC1 脉冲上升/下降沿，TDC(0°) 基准 | 宽度 = (FE−RE) mod 360，中心 = RE + 宽度/2 → `pos_tdc1`, `tdc1_width` |
| `TDC2_RE / _FE` | TDC2 脉冲上升/下降沿，TDC(0°) 基准 | 同上 → `pos_tdc2`, `tdc2_width`（TDC1/TDC2 为两个独立冗余传感器，CH00/CH07） |
| `REF1_TDC_RE / _FE` | REF1 沿位置，TDC(0°) 基准 | 同上 → `pos_ref1`, `ref1_width` |
| `REF2_BDC_RE / _FE` | REF2 沿位置，**BDC(180°) 基准** | 先加 180° 再按 TDC 基准换算 → `pos_ref2`, `ref2_width` |

本台 X1+ 解析结果：REF1/REF2 同处 BDC(180.0°) 宽 1.0°；TDC1/TDC2 中心 0.2°
（RE 359.7° / FE 0.7°）宽 1.0°；A/C 中心 180.0°、B/D 中心 180.3°、宽 0.6°；
180° = 93 整齿周期（186 齿），故 REF 恰落在齿网格上。

验证：`test_x1plus_machine_edges` 以 dt = 2 µs 仿真 3 转，全 8 通道边沿
与 GUI 参数一致（容差 < 1.5 mDeg，采样量化 0.72 mDeg/点），且
C≡A、D≡B、TDC1≡TDC2（CH00≡CH07）、REF2≡REF1。

## 采样率约束

`dt` 必须小于最窄脉冲在最高转速下的持续时间：
`dt < 最窄宽度[deg] / (6 × max|rpm|)`（程序会自动告警）。
例如 0.5 deg 脉冲 @ 180 rpm → 至少 5 µs。

## 文件

```
casim/profile.py    转速曲线 (常数/斜坡/波动/折线)
casim/sensor.py     拾波信号模型 + 安装位置配置 (SensorSpec)
casim/ssi.py        SSI 角度字编码 (12bit + PFB)
casim/generator.py  虚拟飞轮 + 信号生成 (CASimulator, 9474 通道映射)
casim/diagram.py    飞轮安装位置图 (make_diagram)
casim/gui_config.py 实机 GUI (mDeg) → SensorSpec (gui_to_spec)
casim/__main__.py   CLI (--config 支持 GUI / SensorSpec 两种 JSON)
configs/x1plus.json X1+ 试验台 CA Generator 实机参数 (GUI Offsets 面板值)
web/ca_dashboard.html 单页面交互仿真台 (纯前端, 浏览器直接打开; 公式与 casim 一致)
tests/              pytest 验证 (频率/每转脉冲/宽度/安装位置/SSI/积分/X1+ 实机边沿)
tools/precheck_c_port.py  C 固件移植运算顺序位级预检 (已验证与 casim 一致)
HANDOFF.md          项目交接文档 (状态/公式/位级基线/固件移植计划与故障注入设计, 供 AI 续接)
firmware/           嵌入式 C 固件 (移植中: 核心算法 + 故障注入 FR-30~34 + 黄金位级测试)
```

## 与真实主机的对应

- 真机上 A~D/REF/TDC 由飞轮拾波传感器产生，ACM-20 估算 CA 后经 SSI
  送入 ECS；未来该功能将迁入 WiCE ECS。
- 试验台上本工具等价于 CA Generator 的 **Sender 模式**（直接输出 SSI1/2
  时钟+数据）或 **Receiver 模式**（按 pickup 信号估算 CA）。
