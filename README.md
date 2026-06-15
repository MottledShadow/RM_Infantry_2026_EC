# RM_Infantry_2026

RoboMaster 2026 赛季步兵机器人下位机控制固件，基于 STM32F407 + HAL 库。

代码采用清晰的分层架构（BSP → 驱动 → 控制 → 应用），所有子模块初始化收敛到唯一入口
`App_Init()`，便于阅读、移植和二次开发。

> 本仓库已将 CubeMX 生成的 `Drivers/`（HAL + CMSIS）一并纳入版本控制，
> **clone 下来用 Keil 直接打开即可编译，无需安装 CubeMX、无需重新生成代码。**

## 硬件平台

| 项目     | 说明                                              |
| -------- | ------------------------------------------------- |
| 主控     | STM32F407IGHx（大疆 RoboMaster A 型开发板）        |
| IMU      | BMI088（SPI1）                                     |
| 遥控器   | DT7/DR16（DBUS，USART3）                           |
| 裁判系统 | RM 裁判系统串口（USART6）                          |
| 图传     | VTM 图传链路串口（USART1）                         |
| 电机     | CAN1 / CAN2（RM 电机 + C620/C610 电调，GM6020 等） |

## 工具链版本

| 工具           | 版本                                                        |
| -------------- | ----------------------------------------------------------- |
| Keil MDK-ARM   | V5.32 及以上                                                |
| 编译器         | **AC5（ARM Compiler 5.06 update 7）**                       |
| STM32CubeMX    | 6.16.0（仅在需要修改硬件配置时使用）                         |
| 固件包         | STM32Cube FW_F4 V1.28.3                                     |

> ⚠️ **编译器注意**：本工程使用 **AC5（ARMCC）** 编译。新版 Keil MDK（5.37+）默认不再自带 AC5，
> 若你的 Keil 编译报大量错误，请确认已安装 AC5，或在
> `Options for Target → Target → ARM Compiler` 中切换并迁移到 AC6。

## 快速开始

### 方式一：直接编译（推荐，无需 CubeMX）

```bash
git clone <your-repo-url>
```

1. 用 Keil MDK 打开 `MDK-ARM/project.uvprojx`；
2. `Project → Build`（F7）编译；
3. 连接 ST-Link / J-Link，`Flash → Download`（F8）烧录。

因为 `Drivers/` 已经入库，这一步**完全不需要打开 CubeMX**。

### 方式二：修改硬件配置后重新生成

仅当你要改引脚 / 时钟 / 外设时才需要：

1. 用 **CubeMX 6.16.0** 打开 `project.ioc`（确保已安装 **FW_F4 V1.28.3**）；
2. 修改配置后点击 `GENERATE CODE`；
3. 重新生成只会改动 `Core/` 和 `Drivers/`，`MDK-ARM/User/` 下的业务代码不受影响。

## 软件架构

```
应用层    App_Init()  ── 唯一初始化入口，管理全部子模块启动顺序
          referee_ui  ── 裁判系统 UI 绘制
          ───────────────────────────────────────────────
控制层    gimbal / chassis / shooter / friction_wheel
          heat_control / pid / lpf / filter
          ───────────────────────────────────────────────
驱动层    dt7_driver(遥控) / rsi_driver(裁判) / vtm_driver(图传)
          motor_driver / BMI088driver
          ───────────────────────────────────────────────
BSP 层    bsp_can / bsp_uart / bsp_pwm / bsp_gpio
          bsp_flash / bsp_debug / BMI088Middleware
          ───────────────────────────────────────────────
HAL/CMSIS Core/ + Drivers/（CubeMX 生成）
```

控制循环由 **TIM1（1ms）** 中断驱动，依次调用 `Gimbal_Control / Chassis_Control /
Shooter_Control`。初始化顺序约束（IMU 标定、裁判系统握手、摩擦轮电调握手等）
全部集中在 [`MDK-ARM/User/app.c`](MDK-ARM/User/app.c)，内有详细中文注释。

## 目录结构

```
RM_Infantry_2026/
├── Core/                 # CubeMX 生成：main、外设初始化、中断、HAL 配置
├── Drivers/              # CubeMX 生成：HAL 库 + CMSIS（已入库）
├── MDK-ARM/
│   ├── project.uvprojx   # Keil 工程文件
│   ├── startup_*.s       # 启动文件
│   └── User/             # ★ 全部业务代码（BSP / 驱动 / 控制 / 应用）
├── project.ioc           # CubeMX 工程配置（硬件唯一真源）
├── .gitignore
├── LICENSE
└── README.md
```

## 外设分配

| 外设       | 用途                          |
| ---------- | ----------------------------- |
| TIM1       | 1ms 控制循环中断              |
| TIM8 CH1/2 | 摩擦轮 PWM                     |
| SPI1       | BMI088 IMU                    |
| CAN1/CAN2  | 电机收发                      |
| USART1     | 图传 VTM                      |
| USART3     | 遥控器 DT7（DBUS）            |
| USART6     | 裁判系统                      |

## 贡献

欢迎 Issue 与 PR。提交前请保持现有分层结构与中文注释风格一致；若改动了硬件配置，
请同步提交更新后的 `project.ioc`，并在 PR 说明中注明所用的 CubeMX / 固件包版本。

## License

[MIT](LICENSE) © 2026 MOS
