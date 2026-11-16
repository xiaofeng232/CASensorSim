# CASensorSim 固件 — CA 传感器信号仿真器

WinGD X 系列主机试验台的 CA（曲轴角）传感器信号仿真器**固件**：以发动机转速为
输入，通过虚拟飞轮实时生成与真机一致的传感器时序（8 路数字拾波
TDC1/REF1/A/B/REF2/C/D/TDC2 + 双通道 SSI 绝对角度），供主机控制软件
（WiCE ECS / ACM-20）台架开发、测试与故障诊断验证。

本仓库**只含固件 app 代码**（Zephyr，目标板 NUCLEO-F429ZI，与试验台 KiCad
硬件同型）。项目描述、`casim` Python 黄金模型、web 仿真台与需求/设计文档在
平台工作区 `doc/ca sensor sim/`——其中 `HANDOFF.md` 是固件移植的交接文档
（位级一致要求、黄金哈希基线、故障注入需求 FR-30~FR-34）。

## 构建

在 zephyrproject 工作区内（本仓库注册为 west 项目 `apps/cas-sim`）：

```sh
west build -b nucleo_f429zi apps/cas-sim
west flash -d apps/cas-sim/build
```

## 目录结构

```
cas/                     虚拟 CA 引擎库（平台无关，host 直接编译同一份源码）
├── include/             对外头文件：cas_core.h（API）、cas_preset.h（预置参数）
├── src/                 实现：cas_core.c（虚拟飞轮 + 8 路方波 + 双 SSI）
└── CMakeLists.txt       zephyr_library()；app 经 add_subdirectory() 挂载
src/main.c               app 层（板级 bring-up，后续接定时器/GPIO 输出）
test/                    主机黄金测试（纯 C11，不依赖 Zephyr：make -C test test）
```

## 状态

- [x] 板级 bring-up 骨架（LED 心跳）
- [x] `cas_core` 虚拟 CA 核心位级移植（虚拟飞轮 + 8 路方波 + 双 SSI；
      `test/test_golden.c` 黄金哈希与 HANDOFF.md §6 全部一致）
- [ ] `cas_fault` 故障注入（FR-30~FR-34，HANDOFF.md §7.4）
- [ ] 定时器驱动 `cas_step()` + GPIO/SSI 引脚输出（引脚映射取自 KiCad 工程）
- [ ] 参数接口（预置切换 / shell / 上位机在线改参）
