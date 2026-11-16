# CASensorSim 固件 — CA 传感器信号仿真器

WinGD X 系列主机试验台的 CA（曲轴角）传感器信号仿真器**固件**：以发动机转速为
输入，通过虚拟飞轮实时生成与真机一致的传感器时序（8 路数字拾波
TDC1/REF1/A/B/REF2/C/D/TDC2 + 双通道 SSI 绝对角度），供主机控制软件
（WiCE ECS / ACM-20）台架开发、测试与故障诊断验证。

本仓库**只含固件 app 代码**（Zephyr，目标板 FK407M2-ZGT6 / STM32F407ZGT6）。
项目描述、`casim` Python 黄金模型、web 仿真台与需求/设计文档在平台工作区
`doc/ca sensor sim/`——其中 `HANDOFF.md` 是固件移植的交接文档（位级一致要求、
黄金哈希基线、故障注入需求 FR-30~FR-34）。

## 构建

在 zephyrproject 工作区内（本仓库注册为 west 项目 `apps/cas-sim`；板为
out-of-tree，CMakeLists 已把工作区根加入 BOARD_ROOT）：

```sh
west build -p always -b fk407m2_zgt6 apps/cas-sim
west flash -d apps/cas-sim/build
```

## 目录结构

```
cas/                     虚拟 CA 引擎库（平台无关，host 直接编译同一份源码）
├── include/             对外头文件：cas_core.h / cas_fault.h / cas_preset.h
├── src/                 实现：cas_core.c（虚拟飞轮+8 路方波+双 SSI）、
│                        cas_fault.c（故障注入 FR-30/31/33/34）
└── CMakeLists.txt       源码并入 app 镜像（add_subdirectory 挂载）
main.c                   app 层：TIM8 10 µs tick 驱动 cas_step()+cas_fault_apply(),
                         PE0..PE7 = CH00..CH07 数字输出（oe=0 高阻）
boards/                  每板 overlay：fk407m2_zgt6（console→USART1 + TIM8 counter）、
                         nucleo_f429zi（保留可构建）
test/                    主机黄金测试（纯 C11，不依赖 Zephyr：make -C test test）
```

## 信号路径（当前实现）

- **tick**：TIM8 update @ 100 kHz（10 µs，KiCad 引脚分配表）；ISR 内
  `cas_step()` → `cas_fault_apply()` → GPIO 输出，主线程仅 LED 心跳。
- **上电状态**：X1+ 预置（186 齿），const **0 rpm**（静止；转速由后续参数
  接口给出），故障全关（FR-34）。
- **数字输出**：PE0..PE7（24 V 高侧驱动前的 3.3 V 控制脚），`dig[]` 电平 +
  `oe[]` 使能（0 = 切输入高阻，FR-30/HR-02），8 路一次 BSRR 写入。
- **SSI**：双通道字每 tick 已计算（含故障偏移/恒定/单帧/PFB/失效），引脚层
  （PA8/PC0 + RS-485 DIR、TIM6 协议时序）为后续步骤。
- **故障注入**：上电全关（FR-34）；触发条件 立即/作用角/每 n 转/每 n 帧（FR-33）。

## 状态

- [x] 板级 bring-up 骨架（LED 心跳）
- [x] `cas_core` 虚拟 CA 核心位级移植（虚拟飞轮 + 8 路方波 + 双 SSI；
      `test/test_golden.c` 黄金哈希与 HANDOFF.md §6 全部一致）
- [x] `cas_fault` 故障注入（FR-30/31/33/34，HANDOFF.md §7.4；故障向量全过，
      全关时与无故障基线位级一致；缺齿 FR-32 二期）
- [x] 定时器驱动 `cas_step()` + 数字通道 GPIO 输出（TIM8 @ 10 µs，PE0..PE7）
- [ ] SSI 引脚输出（PA8/PC0 + DIR，TIM6 协议时序）
- [ ] 参数接口（预置切换 / shell / 上位机在线改参）
