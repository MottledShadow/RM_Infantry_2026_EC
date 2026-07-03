# RM_Infantry_2026_EC

RoboMaster 2026 赛季步兵机器人下位机控制固件，基于 STM32F407 + HAL 库。

代码采用清晰的分层架构（BSP → 驱动 → 控制 → 应用），所有子模块初始化收敛到唯一入口
`App_Init()`，便于阅读、移植和二次开发。

> 本仓库已将 CubeMX 生成的 `Drivers/`（HAL + CMSIS）一并纳入版本控制，
> **clone 下来用 Keil 直接打开即可编译，无需安装 CubeMX、无需重新生成代码。**

## 硬件平台

| 项目     | 说明                                              |
| -------- | ------------------------------------------------- |
| 主控     | STM32F407IGHx（大疆 RoboMaster C 型开发板）        |
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
全部集中在 [`User/App/app.c`](User/App/app.c)，内有详细中文注释。

## 目录结构

```
RM_Infantry_2026/
├── User/                 # ★ 全部业务代码（按架构分层）
│   ├── BSP/             # 板级支持：CAN/UART/PWM/GPIO/Flash + IMU 中间层
│   ├── Driver/          # 设备驱动：遥控器/裁判系统/图传/电机/BMI088
│   ├── Algorithm/       # 通用算法：PID/低通滤波/CRC/按键扫描
│   ├── Control/         # 运动控制：云台/底盘/发射/摩擦轮/热量
│   └── App/             # 应用层：App_Init 初始化入口 / 裁判系统 UI
├── Core/                 # CubeMX 生成：main、外设初始化、中断、HAL 配置
├── Drivers/              # CubeMX 生成：HAL 库 + CMSIS（已入库）
├── MDK-ARM/
<<<<<<< HEAD
│   ├── project.uvprojx   # Keil 工程文件（含 ../User/* 引用与搜索路径）
│   └── startup_*.s       # 启动文件
=======
│   ├── project.uvprojx   # Keil 工程文件
│   ├── startup_*.s       # 启动文件
│   └── User/             # 全部业务代码（BSP / 驱动 / 控制 / 应用）
>>>>>>> d3eeafb4d3db6ff019e665a574fac925df573e83
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

## License

[MIT](LICENSE) © 2026 MOS
