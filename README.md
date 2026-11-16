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

# 调试变体：日志走 SEGGER RTT（SWD），USART1 保持 fibre 独占
west build -p always -b fk407m2_zgt6 apps/cas-sim -d apps/cas-sim/build-rtt \
    -- -DEXTRA_CONF_FILE=rtt.conf
```

## 目录结构

```
cas/                     虚拟 CA 引擎库（平台无关，host 直接编译同一份源码）
├── include/             对外头文件：cas_core.h / cas_fault.h / cas_preset.h
├── src/                 实现：cas_core.c（虚拟飞轮+8 路方波+双 SSI）、
│                        cas_fault.c（故障注入 FR-30/31/33/34）
└── CMakeLists.txt       源码并入 app 镜像（add_subdirectory 挂载）
src/                     app 层（C++）
├── main.cpp             TIM8 10 µs tick 信号路径 + Fibre 对象（CasObject）
├── uart_backend.{hpp,cpp}  Fibre UART 传输后端（fibre-uart alias）
└── fibre_libc_fix.h     fibre 模块 -include 重定向（补 ssize_t）
interface/               cas_interface.yaml —— CasIntf 接口定义（唯一事实源）；
                         interfaces.hpp/endpoints.cpp/static_exports.cpp 构建期
                         由 modules/lib/fibre 的生成器产出，不入库
host/                    cas_fibre_test.py —— 上位机串口验证脚本
boards/                  每板 overlay：fk407m2_zgt6（fibre-uart=usart1 + TIM8）、
                         nucleo_f429zi（fibre-uart=usart3，保留可构建）
test/                    主机黄金测试（纯 C11，不依赖 Zephyr：make -C test test）
rtt.conf                 SEGGER RTT 日志变体（-DEXTRA_CONF_FILE=rtt.conf）
```

## 信号路径（当前实现）

- **tick**：TIM8 update @ 100 kHz（10 µs，KiCad 引脚分配表）；ISR 内
  `cas_step()` → `cas_fault_apply()` → GPIO 输出 → 遥测镜像刷新，
  主线程仅 LED 心跳。
- **上电状态**：X1+ 预置（186 齿），const **0 rpm**（静止；转速经 fibre
  `set_rpm` 下发），故障全关（FR-34）。
- **数字输出**：PE0..PE7（24 V 高侧驱动前的 3.3 V 控制脚），`dig[]` 电平 +
  `oe[]` 使能（0 = 切输入高阻，FR-30/HR-02），8 路一次 BSRR 写入。
- **SSI**：双通道字每 tick 已计算（含故障偏移/恒定/单帧/PFB/失效），引脚层
  （PA8/PC0 + RS-485 DIR、TIM6 协议时序）为后续步骤。
- **故障注入**：上电全关（FR-34）；触发条件 立即/作用角/每 n 转/每 n 帧（FR-33）。

## 参数/遥测接口（Fibre over USART1）

USART1（PA9/PA10，115200 8N1）专用于 [Fibre](https://github.com/xiaofeng232/fibre)
协议：console/shell 全部关闭，日志走 RTT 变体。接口 `CasIntf`
（`interface/cas_interface.yaml`，构建期代码生成）：

| 属性（只读遥测，每 tick 刷新） | 类型 | 说明 |
|---|---|---|
| `firmware_version` | uint32 | 0x00010000（v1.0.0） |
| `tick_hz` | uint32 | 100000 |
| `rpm` | float32 | 当前瞬时转速（profile 在 t 时刻值） |
| `theta_deg` | float32 | 飞轮角（未绕回，deg） |
| `dig_mask` / `oe_mask` | uint8 | CH00..07 电平 / 输出使能位掩码 |
| `ssi1` / `ssi2` | int32 | SSI 字（12 bit 角度 \| PFB<<12） |
| `sample_count` | uint64 | tick 计数 |

| 函数 | 说明 |
|---|---|
| `set_rpm(rpm_mrpm)` | 切 CONST 转速，单位 **milli-rpm**（60000 = 60.000 rpm，负值反转） |
| `set_dig_fault(ch, enable, trigger, angle_mdeg, every_n)` | FR-30 断线（oe=0/dig=0） |
| `set_ssi_fault(ch, enable, trigger, angle_mdeg, every_n, pfb, offset_mdeg, hold, single_frame_delta, fail)` | FR-31 效果集，可组合 |
| `clear_faults()` | 回到 FR-34 全关状态 |

`trigger`：0=立即，1=作用角（angle_mdeg），2=每 n 转（every_n），3=每 n 帧。
legacy fibre01 端点协议的函数载荷只支持定长 int/uint——bool 用 uint8（0/1）、
转速用 milli-rpm 表达；遥测是属性（property），float32/int32/uint64 原生支持。

并发模型：ISR 每 tick 写遥测成员（32 位对齐存储原子；sample_count 64 位读
可能撕裂，单调计数可接受）；参数写入在 fibre event-loop 线程用 `irq_lock()`
保护，ISR 不会看到半更新的 profile/故障集。

上位机验证（USB-UART 适配器接 PA9/PA10）：

```sh
pip install pyserial
python3 host/cas_fibre_test.py /dev/ttyUSB0        # 全部检查项
```

## 状态

- [x] 板级 bring-up 骨架（LED 心跳）
- [x] `cas_core` 虚拟 CA 核心位级移植（虚拟飞轮 + 8 路方波 + 双 SSI；
      `test/test_golden.c` 黄金哈希与 HANDOFF.md §6 全部一致）
- [x] `cas_fault` 故障注入（FR-30/31/33/34，HANDOFF.md §7.4；故障向量全过，
      全关时与无故障基线位级一致；缺齿 FR-32 二期）
- [x] 定时器驱动 `cas_step()` + 数字通道 GPIO 输出（TIM8 @ 10 µs，PE0..PE7）
- [x] 参数/遥测接口：Fibre over USART1（CasIntf；转速/故障在线下发 +
      遥测镜像；`host/cas_fibre_test.py` 上位机验证）
- [ ] SSI 引脚输出（PA8/PC0 + DIR，TIM6 协议时序）
- [ ] 预置/几何参数在线下发（cfg 级 cas_init 重建；当前仅转速+故障在线）
