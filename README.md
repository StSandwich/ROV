# ROV

> **本项目 fork 自 [FPJ-GAOGE / FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform](https://github.com/FPJ-GAOGE/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform)，
> 由 [@StSandwich](https://github.com/StSandwich) 独立维护，已不再与上游同步。**
> 
> This is a fork of **FinsROV — an Underwater Camera-Based Multi-Robot Platform**,
> maintained independently by [@StSandwich](https://github.com/StSandwich).
> It is no longer kept in sync with the upstream repository.

## 仓库结构 / Repository Structure

```text
ROV/
├── Code/                              # 全部代码
│   ├── Joystick_Control/              # 上位机（Python）：手柄串口通信 + USB 摄像头
│   │   ├── serialPort.py
│   │   ├── usbCameraFullControl.py
│   │   └── readme.md
│   └── Lower_Level_Controller/        # 下位机：STM32F407IGHx（DJI RoboMaster C 板）
│       ├── CubeMX/                    # STM32CubeMX 工程 + MDK-ARM(Keil) 工程
│       │   └── MDK-ARM/
│       ├── Drivers/                   # ST HAL 库 + CMSIS
│       ├── Middlewares/               # ST USB Device 库
│       ├── Inc/  Src/                 # CubeMX 自动生成的初始化代码
│       ├── MDK-ARM/                   # Keil MDK 工程文件
│       ├── startup/                   # 启动汇编与链接脚本
│       ├── userCode/                  
│       │   ├── algorithms/            # 控制 / 姿态解算等算法
│       │   ├── bsp/                   # 板级支持（CAN、串口、定时器…）
│       │   ├── devices/               # 器件驱动（电机、舵机、传感器…）
│       │   ├── drivers/               # 底层外设驱动
│       │   └── MiddleWares/           # 自定义中间件
│       ├── CMakeLists.txt
│       ├── RM_Frame_C.ioc             # CubeMX 配置（改外设时用 CubeMX 打开）
│       └── README.md
├── Making_Instructions/               # 装配说明文档与配图
├── Structure/                         # 结构件：STEP 三维模型与 BOM
│   ├── En/                            # 英文版：3D 打印 / 密封舱 / CNC 加工 / 舵机 / 推进器 / 水深传感器
│   ├── zh-CN/                         # 中文版：同上
│   └── Images/
├── .gitignore
├── .gitattributes
├── LICENSE                            # GPL-3.0（本项目全部内容）
├── LICENSE-MIT                        # 原始项目的 MIT 许可（保留署名）
└── README.md
```

## 许可协议 / License

**本项目整体以 GNU General Public License v3.0 发布**，全文见 [`LICENSE`](./LICENSE)。

```text
Copyright (C) 2026 FPJ-GAOGE      — 原始项目作者
Copyright (C) 2026 StSandwich     — 本项目修改部分
```

本项目是 [FPJ-GAOGE/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform](https://github.com/FPJ-GAOGE/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform) 的**修改版本（fork）**。
原项目的**代码**以 MIT 许可发布，其署名与许可声明完整保留于 [`LICENSE-MIT`](./LICENSE-MIT)。

原项目中 `Structure/`（三维模型、BOM）与 `Making_Instructions/`（文档、配图）等**非代码资源**，原以 **Creative Commons Attribution-ShareAlike 4.0 International（CC BY-SA 4.0）** 发布，原作者署名为 FPJ-GAOGE（<https://creativecommons.org/licenses/by-sa/4.0/>）。本项目已对其作删减与修改。上述非代码资源经改编后，在本项目中**与代码一并统一以 GPL-3.0 发布**。

再分发本项目的任何部分时，请保留本段声明以及 [`LICENSE`](./LICENSE)、[`LICENSE-MIT`](./LICENSE-MIT) 两个文件。
兼容许可清单见 <https://creativecommons.org/compatible-licenses>。
