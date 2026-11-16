# CASensorSim 项目交接文档（供 AI 续接工作）

> 更新时间：2026-11-13 · 状态：**嵌入式 C 固件移植进行中**（含故障注入，新需求）
> 约定：**所有面向用户的回复用中文**。用户消息通常是简短祈使句，指对现有交付物的迭代要求。

---

## 1 项目概览

WinGD X 系列主机试验台的 **CA（曲轴角度）传感器信号仿真器**。试验台没有真机，
本系统以发动机转速为输入，通过虚拟飞轮实时生成与真机一致的传感器时序
（8 路数字拾波 + 双通道 SSI 绝对角度），供主机控制软件（WiCE ECS / ACM-20）
台架开发、测试与故障诊断验证。

四大交付物：

| # | 交付物 | 状态 |
|---|--------|------|
| 1 | `casim` Python 黄金模型 | ✅ 完成（19 项 pytest 通过） |
| 2 | `web/ca_dashboard.html` 单页面交互仿真台（含故障注入使能 UI） | ✅ 完成（浏览器全功能验证 + 位级一致 + 故障回归） |
| 3 | 嵌入式文档（需求规格书 V0.2 / 设计文档 V0.2，MCU 平台，取消 NI） | ✅ 完成 |
| 4 | **`firmware/` C 固件移植（含传感器/SSI 故障注入）** | 🔄 **进行中（当前活动任务）** |

## 2 环境与工具

- Windows PowerShell；工作目录
  `C:\Users\XLI108\OneDrive - WinGD\Documents\Default Project\CASensorSim`；**非 git 仓库**。
- Python 3.10.8 + numpy 2.2.6 + matplotlib + pytest 9.0.2。
  控制台 GBK：CJK 输出用 `python -X utf8 -c "..."`。
- **WSL Ubuntu 24.04 内 gcc 13.3.0 可用**（构建/运行 C 主机测试用）：
  `wsl -e bash -c "gcc --version"`。Windows 侧无主机 gcc（`C:\build\17.0\...` 里只有
  NILRT 交叉 gcc，产物是 Linux ELF，不能直接运行）。
- WSL 访问项目文件：`/mnt/c/Users/XLI108/OneDrive - WinGD/Documents/Default Project/CASensorSim`
  （**路径含空格**，bash 里必须加引号；/mnt/c 稍慢但可用。若有问题先
  `cp -r` 到 `~/casim_fw` 构建再拷回）。
- 网页验证：`python -m http.server 8377`（从 CASensorSim 目录**后台**起，
  **不跨会话保留，每个新会话先重启**）+
  浏览器 tab `http://localhost:8377/web/ca_dashboard.html`。
  **`file://` 不被浏览器工具支持**。
- 浏览器工具坑：
  - `browser.evaluate` 结果在 `.value`；
  - `browser.wait` 对快速变化文本（实时状态行）不可靠 → 用数值探针（evaluate 读
    `RUN.state`/`RUN.t`/DOM 文本）；
  - `files.get` ≤ 5 MiB（25 万行 CSV 传不回，用页内采样/哈希）；
  - rAF 只在窗口可见时触发（页面已内置 16 ms `setInterval` 回退，后台窗口也能推进）；
  - **screenshot 需要窗口可见**，失败时改用数值探针验证（2026-11-13 实测）。
- OneDrive 同步：写文件无问题；勿依赖 git。

## 3 物理模型与公式（严格遵循，勿改）

- **单飞轮单齿轮**：A/B/C/D 四路拾波共用同一 `teeth` 与 `tooth0`；齿周期 P = 360/teeth。
- **TDC1/TDC2 是两个独立冗余传感器**，同读飞轮上唯一的 `tdc_mark`（TDC1→CH00，TDC2→CH07）。
- REF1/REF2 各读自己的标记，每转 1 脉冲。
- **NI 9474 接线（2 组 × 4）**：
  `CH00 TDC1, CH01 REF1, CH02 A, CH03 B, CH04 REF2, CH05 C, CH06 D, CH07 TDC2`。
- 公式（**运算顺序影响位级一致，见 §7.3**）：
  - 转速→角度：梯形积分 `θ[k+1] = θ[k] + 0.5·(rpm[k]+rpm[k+1])·6·dt`（deg）
  - 脉冲中心：齿拾波 `(pos − tooth0) mod (360/teeth)`；标记 `(pos − mark) mod 360`
  - 方波：`rel = (θ − center + L/2) mod P`；`level = (rel < L)`
  - SSI：`a = (θ + off) mod 360`；`count = round_banker((a/360)·4096) mod 4096`；
    `bit12 = PFB`；13 bit 帧（12 bit 角度 MSB first + PFB）
- **角度单位约定（重要）**：
  - 界面/配置的所有角度参数一律 **mDeg（1/1000°，0–360000）**，与 X1+ GUI 一致；
  - 内部 ÷1000 转 deg 参与计算；CA 显示（图/读数）按 **0–360000 mDeg 每转回绕**（锯齿）；
  - GUI 配置 JSON（`configs/x1plus.json`）为 mDeg（`numOffTeeth` 等 GUI 字段名）；
  - CLI `--dt` 单位 **µs**（如 `--dt 20`）；
  - CSV 黄金列格式 `t_s,angle_deg,speed_rpm,CH00..CH07,ssi1,ssi2`（`angle_deg` 是 deg——
    数据文件约定，界面不显示该列）。
- 转速曲线：const / ramp（start→target，accel，可 cycle 往复）/ fluct（base + amp·sin(2πf·t)）/ 折线。
- 负转速（FR-04）：θ 递减，公式不变（casim 有反转测试）。

## 4 文件地图

```
CASensorSim/
├── HANDOFF.md                  ← 本文档
├── README.md                   主文档（用法/参数/文件清单/对应真机说明）
├── casim/
│   ├── profile.py              转速曲线 (const/ramp/fluct/curve)
│   ├── sensor.py               方波模型 + SensorSpec（centers()/signals()）
│   ├── ssi.py                  SSI 编码 (SSISpec._counts: np.round 银行家舍入)
│   ├── generator.py            CASimulator：WIRING、梯形积分（L86-89 是关键）
│   ├── gui_config.py           GUI(mDeg) → SensorSpec (gui_to_spec)
│   ├── diagram.py              飞轮安装图
│   └── __main__.py             CLI（--config/--rpm/--duration/--dt(µs)/--out/--plot）
├── configs/x1plus.json         X1+ 实机 GUI 参数（mDeg）
├── web/ca_dashboard.html       单页面仿真台（纯前端 ~1000 行，详见 §8）
├── tests/test_signals.py       19 项 pytest（含 X1+ 实机边沿 test_x1plus_machine_edges）
├── tools/precheck_c_port.py    C 移植运算顺序预检（✅ 已跑通，输出见 §6）
├── out/                        示例输出（x1plus.csv 默认 dt=50µs；位级比对用 --dt 20）
└── docs/
    ├── 嵌入式CAS控制系统需求规格书.md   V0.2（FR/PR/HR/RR/VR/NFR；FR-30~34 = 故障仿真）
    └── 嵌入式CAS控制系统设计文档.md     V0.2（§5 关键算法逐条对应 casim 模块；§5.5 黄金比对）
```

**`firmware/` 尚不存在**——待建，结构见 §7.2。

## 5 已完成并验证的事实

- `python -m pytest tests -q` → **19 通过**。
- Web 静态预览与 casim **25 万样本位级一致**（页内 vs `python -m casim --config
  configs/x1plus.json --rpm 60 --duration 5 --dt 20 --out ...`，h31 哈希 + 高电平计数全同）。
- Web 动态引擎全流程浏览器实测：▶运行/⏸暂停/⏹停止、时标 1×/0.25×/0.05×、循环、
  **运行中改转速即时生效**（60→200 一帧内）、停止后静态预览按当前参数重建；
  窗口隐藏时 16 ms interval 回退驱动正常推进（实测 t=22.7 s 连续）。
- 画布随卡片铺满（waveW 动态宽度 + resize 自适应）。
- 角度参数全部 mDeg 化后复验：哈希仍与 §6 黄金值**完全一致**
  （180300/1000 等 mDeg→deg 换算与 Python 字面量同 double）。
- **C 移植运算顺序预检通过**（2026-11-13，`tools/precheck_c_port.py`）：
  `t = k·dt` 与 `np.arange` 位级相同；顺序 `cum += inc; θ = start + cum` 与
  `start + np.cumsum(inc)` 位级相同；8 通道 + SSI 全部 `identity=True`，
  哈希/ones 与 §6 黄金常量一致 → **C 移植按 §7.3 顺序实现即可位级复现**。

## 6 位级一致基线（黄金常量，勿改）

测试条件：**X1+ 参数、const 60 rpm、dt = 20 µs、duration = 5 s → N = 250001**。
哈希定义：`h = 0; for v in arr: h = (h*31 + v) mod 2^32`（通道 v∈{0,1}；SSI v=读字）。

| 通道 | 信号 | h31 | 高电平计数 |
|------|------|-----|-----------|
| CH00 = CH07 | TDC1 = TDC2 | 3205541500 | 696 |
| CH01 = CH04 | REF1 = REF2 | 1802195195 | 695 |
| CH02 = CH05 | A = C | 3446791681 | 77501 |
| CH03 = CH06 | B = D | 2104567553 | 77501 |
| SSI1 = SSI2 | — | 1136600576 | — |

- X1+ 解码值（mDeg）：teeth=186；pos_a/pos_c/pos_ref1/pos_ref2 = 180000；
  pos_b/pos_d = 180300；pos_tdc1/pos_tdc2 = 200；tooth0/ref 标记/tdc 标记 = 0；
  宽度 a~d = 600，ref/tdc = 1000。
- 首个 TDC1 边沿为**下降沿** idx=97（t=0.00194 s，θ≈0.6984°）：
  高电平区 [359.7°, 0.7°) 跨 TDC（中心 0.2° 宽 1.0°）。
- 边沿期望（mDeg mod 360000，tol 1.5 @dt=2 µs，pytest 内已固化）：
  A RE 179700+k·1935.48，B RE 180000+…，REF1/2 RE 179500 / FE 180500，TDC1/2 RE 359700 / FE 700。
- Python 复算：`python -m casim --config configs/x1plus.json --rpm 60 --duration 5 --dt 20 --out out/_check.csv`
- 预检脚本输出（2026-11-13 实测，全 True）：`python -X utf8 tools/precheck_c_port.py`

## 7 活动任务：C 固件移植（含故障注入）

### 7.1 范围确认
- MCU 设备 = **信号发生器**（需求规格书 §1：嵌入式设备实时生成全套 CA 传感器信号；
  Sender/Receiver 模式见 FR-41）。移植对象 = casim 黄金模型算法（NFR-02 公式一致）。
- VR-05：MCU 侧以黄金模型**位级比对**复测（本文档 §6 即基线；比对脚本为"既定跟进项"
  → 本次实现 = `firmware/test/test_golden.c`）。
- **用户新增要求（2026-11-13）：传感器故障注入 + SSI 故障** → 对应需求规格书
  FR-30~FR-34（已定义），在核心算法之上加故障注入模块（§7.4）。
  注：web 仿真台故障注入 UI 已于 2026-11-13 实施（断线/两组 CA 差值/SSI 恒定/PFB/
  总失效，详见 §8）；固件侧仍按本节 cas_fault 模块实现（含触发条件 FR-33、缺齿 FR-32
  二期），两者语义对齐。

### 7.2 目录结构（待建）
```
firmware/
├── cas_core.h / cas_core.c     核心算法（位级一致，§7.3）
├── cas_fault.h / cas_fault.c   故障注入（FR-30~FR-34，§7.4）
├── cas_preset.h                默认 100 齿 / X1+ 186 齿预置（mDeg，与 web PRESETS 一致）
├── platform/cas_hal.h          HAL 接口（8 路电平+高阻、SSI 字/时序回调）
├── platform/stm32h743_hal.c    STM32H743 参考骨架（GPIO/10µs TIM/RS-485 DIR，标 TODO）
├── test/test_golden.c          主机黄金测试：基线哈希 + 故障向量（§7.4 末）
├── Makefile                    cc -O2 -std=c11 -Wall -Wextra -I.
└── README.md                   移植说明 / 构建 / 与文档章节对照
```

> **2026-11-16 落位更新**：仓库已迁入 git（github.com/xiaofeng232/CASensorSim），
> **根目录即 Zephyr app**（`CMakeLists.txt` / `prj.conf` / `src/main.c`，
> 板 = NUCLEO-F429ZI；zephyrproject 工作区 west.yml 注册为 `apps/cas-sim`）。
> 上面 `firmware/` + Makefile 结构不再单独建：
> - `cas_core.h/.c`、`cas_fault.h/.c`、`cas_preset.h` → `src/cas/`
> - `platform/` HAL 骨架 → Zephyr GPIO/TIM API 直接实现（目标板 F429，
>   引脚映射取自 KiCad 工程，**不是** §7.2 的 STM32H743）
> - `test/test_golden.c` → twister native 单测（`west twister -T <dir> -p
>   native_sim`），或 plain gcc 手跑亦可（源码平台无关）
> - §7.5 的 WSL+Makefile 构建步骤作废 → `west build -b nucleo_f429zi apps/cas-sim`

### 7.3 核心 API 与位级一致实现要点（已设计，待实现）

```c
/* 通道顺序 = 9474 接线: 0=TDC1 1=REF1 2=A 3=B 4=REF2 5=C 6=D 7=TDC2 */
typedef struct {
  uint16_t teeth;                      /* A/B/C/D 共用 */
  uint32_t tooth0_mdeg, ref1_mark_mdeg, ref2_mark_mdeg, tdc_mark_mdeg;
  uint32_t pos_mdeg[8];                /* CH00..CH07 安装角 */
  uint32_t width_mdeg[8];              /* CH00..CH07 高电平宽度 */
} cas_cfg_t;

typedef enum { CAS_PROF_CONST=0, CAS_PROF_RAMP, CAS_PROF_RAMP_CYCLE, CAS_PROF_FLUCT } cas_prof_type_t;
typedef struct {
  cas_prof_type_t type;
  double rpm, start_rpm, target_rpm, accel_rpm, ret_rpm, amp_rpm, freq_hz;
} cas_profile_t;                       /* 与 casim/profile.py 逐式对应 */

typedef struct { uint32_t ssi1_off_mdeg, ssi2_off_mdeg; bool pfb1, pfb2; } cas_ssi_cfg_t;

typedef struct {
  uint64_t k; double t, theta, rpm;    /* 当前样本 (k: t=k·dt) */
  uint8_t dig[8]; int32_t ssi1, ssi2;  /* 当前输出 */
  uint8_t oe[8];                       /* 通道使能 (0=高阻, FR-30) */
  /* 私有: start, cum, dt, rpm_prev, center[8], period[8], width[8], ssi 偏移 */
} cas_state_t;

void cas_init(cas_state_t*, const cas_cfg_t*, const cas_profile_t*,
              const cas_ssi_cfg_t*, double dt);   /* 算第 0 样本 */
void cas_step(cas_state_t*);                      /* 推进到 k+1（10~20 µs 定时调用） */
double cas_profile_rpm(const cas_profile_t*, double t);
```

**逐式对应 casim（运算顺序已预检通过，见 §5/§6）**：
1. `t = (double)k * dt`（= np.arange start=0 的 `i*step`，勿用累加 t+=dt）。
2. `cum += ((0.5*(rpm_prev + rpm_new)) * 6.0) * dt;  theta = start + cum;`
   （= `inc = 0.5*(rpm[1:]+rpm[:-1])*6.0*dt; angle[1:] = start + cumsum(inc)`；
   start=0 时与逐次 θ+= 相同，start≠0 必须用 start+cum 形式）。
3. init 时预计算 `P = 360.0/(double)teeth`、
   `center[ch] = fmod_pos(pos_mdeg/1000.0 − mark_mdeg/1000.0, period)`
   （齿通道 mark=tooth0、period=P；REF/TDC mark=对应标记、period=360.0）。
4. 每样本：`rel = fmod_pos((theta − center) + 0.5*width, period); level = (rel < width)`。
5. SSI：`a = fmod_pos(theta + off, 360.0); c = rne((a/360.0) * 4096.0);
   word = (int32_t)(c % 4096.0) | (pfb ? 1<<12 : 0);`
6. `fmod_pos(x,m)`: `r=fmod(x,m); if(r<0) r+=m;`（= Python 浮点 %）。
7. `rne` = np.round 银行家舍入：`f=floor(x); d=x−f; d>0.5→f+1; d<0.5→f; d==0.5→f 为偶取 f 否则 f+1`
   （网页 `roundHalfEven` 已按此位级验证过）。
8. const/ramp 纯算术 → 位级一致有保证；fluct 依赖 `sin()`，不同 libm 末位 ulp 可能差
   → 边界边沿可能 ±1 样本（VR-01 容差 ±1 dt 内，文档注明即可）。
9. 故障注入在 `cas_step` 信号算出**之后**、写 `st->dig/oe/ssi1/ssi2` 之前应用（§7.4）。

### 7.4 故障注入模块（FR-30~FR-34，**用户新增要求**）

需求基线（需求规格书 §3.4，已定义，直接实现）：
- FR-30 数字通道断线：任一路实时通/断，**OFF = 高阻**（`oe[ch]=0`，HR-02）；
- FR-31 SSI 故障：PFB 置位；角度永久偏移 / 短时偏移 / 单帧扰动；单通道角度恒定；
  总失效 / 短时失效（高阻，按作用角触发）；
- FR-32 齿轮齿故障：缺齿/坏齿（作用传感器、起始齿号、齿数、缺失宽度 %）；
  附加干扰脉冲（起始角、脉宽、间隔、个数 ≤10）——**建议放二期**，先落 FR-30/31/33/34；
- FR-33 触发条件：立即 / 指定作用角 / 每第 n 转 / 每第 n 帧 SSI 传输；
- FR-34 上电默认全关，显式使能才注入。

```c
typedef enum { CAS_FT_IMMEDIATE=0, CAS_FT_AT_ANGLE, CAS_FT_EVERY_N_REV, CAS_FT_EVERY_N_SSI } cas_fault_trigger_t;

typedef struct {
  bool enable;                 /* FR-34 */
  cas_fault_trigger_t trigger;
  uint32_t angle_mdeg;         /* AT_ANGLE 作用角 */
  uint32_t every_n;            /* EVERY_N_REV / EVERY_N_SSI */
  bool active;                 /* 内部状态 */
} cas_fault_cond_t;

typedef struct { cas_fault_cond_t cond; } cas_dig_fault_t;          /* FR-30: 断线→高阻 */

typedef struct {
  cas_fault_cond_t cond;
  bool pfb;                    /* PFB 置位（可独立常开, 也可挂 cond） */
  int32_t offset_mdeg;         /* 永久/短时角度偏移 */
  bool hold;                   /* 角度恒定（触发时冻结读字） */
  int32_t single_frame_delta;  /* 单帧扰动（仅 1 帧，随后恢复） */
  bool fail;                   /* 总/短时失效（SSI 高阻） */
} cas_ssi_fault_t;                                   /* FR-31 */

typedef struct {
  cas_dig_fault_t dig[8];
  cas_ssi_fault_t ssi[2];
  uint32_t rev_count;          /* 转计数（θ mod 360 上穿 0） */
  uint32_t ssi_frame_count;    /* SSI 帧计数（master 帧 / slave 突发） */
} cas_fault_t;

void cas_fault_reset(cas_fault_t*);                  /* 全关（FR-34 默认态） */
void cas_fault_apply(cas_fault_t*, cas_state_t*);    /* 在 cas_step 尾部调用 */
```

实现要点：
- 触发状态机：`AT_ANGLE` 判 (θ mod 360) 跨入作用角（按每 tick 区间判断，支持负转速方向）；
  `EVERY_N_REV` 在转计数（TDC1 上升沿计数）到达 n 的整数倍时激活/解除（设计为"激活持续"
  或"单次触发"两种，测试向量按"激活持续"取定）；`EVERY_N_SSI` 按 SSI 帧计数。
- 数字断线生效时：`st->dig[ch]=0; st->oe[ch]=0`（HAL 切高阻），其余通道不受影响。
- SSI 各效果作用顺序：先角度偏移/恒定 → 再单帧扰动 → 最后 PFB 位；
  `fail` 生效时 SSI 高阻（由 HAL 表示，核心里置 `st->ssi_fail[ch]=1` 状态位）。
- 全部故障关闭时输出与无故障基线**位级相同**（FR-34 的测试判据）。

**test_golden.c 故障测试向量（确定性，实现时复核具体数值）**：
1. 基线（全关）→ §6 哈希/ones 全同；
2. CH00(TDC1) 断线（IMMEDIATE，t≥2.0 s 段或 AT_ANGLE=0）：CH00 高电平计数 =
   仅 [0, 2 s) 窗口内 = 2/5 × 696 ≈ 278（实现时按样本数精确计算）；`oe[0]=0`；
   其余 7 通道哈希不变；
3. SSI2 PFB 常开：所有 ssi2 读字 bit12=1，低 12 位与基线逐样本相同；
4. SSI2 永久偏移 +500 mDeg：`ssi2[k] = (base_count[k] + round(500/360000·4096)) mod 4096`；
5. SSI2 角度恒定（AT_ANGLE=180000 mDeg 触发）：触发后 ssi2 恒 = 2048（180°→2048 count）；
6. （二期）CH02(A) 缺齿 1 个/转：5 s @60 rpm 少 5 个脉冲 → ones = 77501 − 5 = 77496。

### 7.5 执行清单（按序）
1. ~~运算顺序预检~~ ✅（2026-11-13 通过，输出已固化在 §6 与 tools/precheck_c_port.py）。
2. 写 `firmware/cas_core.h/.c`（§7.3；每条公式注释引用 casim 源文件行号：
   generator.py L86-89 积分、sensor.py L48-49 方波 / L115-131 中心、
   ssi.py L28-30 计数、profile.py 各 speed()）。
3. 写 `firmware/cas_fault.h/.c`（§7.4）+ `cas_preset.h`（X1+ mDeg 值见 §6）。
4. 写 `firmware/test/test_golden.c` + `Makefile`：
   - 单元断言：rne(0.5→0, 1.5→2, 2.5→2, 3.5→4, 67.5→68)、fmod_pos 负值、
     方波边界（θ=center±L/2）、const/ramp profile 值；
   - 基线：250001 样本 8 通道 + SSI 哈希/ones == §6 常量（`-DUNIT` 宏或分 main）。
   - 故障向量：§7.4 的 1~5 项。
5. WSL 构建运行：
   ```
   wsl -e bash -c "cd '/mnt/c/Users/XLI108/OneDrive - WinGD/Documents/Default Project/CASensorSim/firmware' && make test"
   ```
   通过判据：gcc `-Wall -Wextra` 零告警；全部断言 PASS。
6. 文档同步：
   - `docs/嵌入式CAS控制系统设计文档.md` §5.5："比对脚本为既定跟进项" →
     已实现（`firmware/test/test_golden.c` + 结果哈希）；
   - 设计文档新增 §5.6 故障注入（对应 FR-30~FR-34，引 `cas_fault.c`）；
   - 主 `README.md` 文件清单加 `firmware/`、`tools/`、`HANDOFF.md`，用法加一条
     `cd firmware && make test`（WSL）。
7. 向用户中文总结（核心 + 故障注入 + 位级验证结果 + 后续：STM32 骨架/二期缺齿故障）。
8. **更新本 HANDOFF.md**（状态列、新增事实、剩余项）。

### 7.6 验收标准
- 基线 8 通道 + SSI 哈希/ones 与 §6 严格一致（bit-level）；
- 故障向量全过；故障全关时与基线位级相同（FR-34）；
- `-Wall -Wextra` 零告警；核心代码平台无关（不依赖 MCU 头文件），
  主机/目标同一份源码。

## 8 Web 仿真台要点（如需继续 web 侧工作）

- `web/ca_dashboard.html`：纯前端单文件，无服务器/CDN；公式与 casim 位级一致
  （JS `roundHalfEven` 实现 np.round 银行家舍入）。
- 关键全局：`RUN`（stopped/running/paused 状态机）、`ring`（1 s 窗口环形缓冲，
  M=WIN/dt，typed arrays，vi 单调计数）、`CHANS/POSKEY/WKEY/COL`、
  `PRESETS`（**mDeg**）、`readParams()`（mDeg ÷1000 → deg 再入模）、
  `simulate()`（静态预计算）、`readLiveParams()/liveStep()/frame()/drawLiveScope()/
  renderLiveVals()`（动态引擎）、`waveW()`（画布随容器宽度，`#waveDig,#waveCA{width:100%}`）。
- 调度：rAF 主 + 250 ms 未触发 → 16 ms `setInterval` 回退（`driver`='raf'/'interval'）；
  暂停/停止/recompute 时 `stopScheduler()`；窗口 resize：停止态防抖 recompute，
  运行态每帧自适应。
- 运行中可随时改转速/参数（`readLiveParams` 逐帧读字段，非法输入保持上次有效参数；
  `scheduleRecompute` 在非停止态跳过）。
- 验证用探针（窗口隐藏时）：
  `browser.evaluate` 读 `JSON.stringify({state:RUN.state,t:RUN.t,lvA:...,lvR:...})`。
- **故障注入 UI（2026-11-13 已实施，FR-30/31/34 的 web 侧对应）**：
  - HTML：独立「故障注入」fieldset——`fz0`–`fz7` 断线复选框（OFF = 高阻）、
    `fzGrp`+`fzGrpDelta`（两组 CA 差值 Δ，mDeg 可负，平移组 2 CH04–CH07 中心，
    使两组拾波各自估算的 CA 相差 Δ）、`pfb1`/`pfb2`（自原仿真 fieldset 移入，ID 不变）、
    `ssi1_hold`/`ssi2_hold`+`ssi1_hold_ang`/`ssi2_hold_ang`（角度恒定，默认 180000 mDeg）、
    `ssi1_fail`/`ssi2_fail`（总失效=高阻，**覆盖**恒定）。
  - JS：`readFaults()`（返回 f 含 `z[8]`/grp/pfb/hold/fail/`any`）、`sseHold(aDeg)`
    （= roundHalfEven(mod(a,360)/360·4096) mod 4096，与实时 SSI 同计数式）、
    `faultList(f)`（派生表摘要）；`GRP2=Set(['REF2','C','D','TDC2'])`。
  - 数据路径：`recompute` → `faultsRef=readFaults()` → `simulate(p,f)`（组 2 中心
    平移在 centers 计算后、方波生成前；断线 fill(0)+edges=[0]；SSI fail>hold>PFB 顺序；
    返回 `zDig[8]`/`zSsi[2]`）；动态：`readLiveParams` 同式平移 + `p._faults`，
    `liveStep` 逐样本应用并置 `RUN.z={dig,ssi}`（供示波/读数）。
  - 可视化：Z 通道 = 虚线灰平线 +「高阻 (OFF)」标注（静态 buildOffDig + 动态
    drawLiveScope）、chip 虚线灰（`.chip.z`）、SSI 失效时 lvS/lvP 显示「–」、
    SSI1 失效时 CA 图不画锯齿改标注；CSV 导出附「（含故障注入，高阻=0）」。
  - **位级一致保持**：全部故障关闭时 `centers`/`dig`/`ssi` 计算路径与无故障完全相同
    （组 2 平移与断线均有 `if` 守卫）；浏览器回归实测 10 项哈希 == §6 黄金值。
  - `applyPreset` 复位：PRESETS 已含全部故障字段（fz*、fzGrp/fzGrpDelta、pfb*、
    ssi*_hold/_ang/_fail），切预设即恢复默认全关。
  - 未实现（web 侧，固件 §7.4 有）：触发条件（AT_ANGLE/每第 n 转/每第 n 帧，FR-33）、
    单帧扰动、缺齿（FR-32）——web 目前均为"使能即持续生效"。
- 当前页面状态（2026-11-13 末）：stopped，X1+ 预置，rpm=60，故障全关。

## 9 早先提议、用户未选择的跟进项（勿擅自开工）

- `tools/identify.py`（实测数据 → 传感器参数辨识）；
- `configs/schema.json`（GUI JSON 校验）；
- 上位机通信协议（M8）字节级设计（需求规格书 HR-06 以太网接口）。

## 10 沟通与工作方法

- 回复用中文；结论先行，改动点列清楚。
- 改动 `ca_dashboard.html` 后必须浏览器重载 + 控制台错误检查 + 数值探针/截图验证
  （截图需要窗口可见，失败就换数值探针）。
- 位级一致类改动：改完必须重跑 §6 哈希比对（web 用页内 hash；firmware 用
  test_golden.c；Python 用 tools/precheck_c_port.py 或 casim 重算）。
- 文档（README/需求规格书/设计文档）与代码同步更新；需求编号（FR/VR/NFR）引用要准确。
