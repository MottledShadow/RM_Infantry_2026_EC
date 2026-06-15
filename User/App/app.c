/**
 ******************************************************************************
 * @file    app.c
 * @brief   应用层统一初始化实现
 * @note    【核心职责】
 *            这是整个工程 Init 调用的唯一出口。
 *            所有子模块的 Init 函数调用及顺序约束全部集中在此文件，
 *            消除原版"Init 套 Init、顺序混乱"的问题。
 *
 *          【初始化顺序与约束说明】
 *
 *          ─────────────────────────────────────────────────────────────────
 *          阶段一：BSP 层硬件启动（无顺序约束，均依赖 MX_xxx_Init 已完成）
 *          ─────────────────────────────────────────────────────────────────
 *          1. BSP_UART_StartReceive();
 *             → 启动所有 UART DMA 接收（UART 必须最先启动！
 *               否则后续等待裁判系统数据的循环永远无法收到数据）
 *             → 注意：不启动控制定时器中断，中断必须等所有 Init 完成后才能启动
 *
 *          2. can_filter_init()
 *             → 配置 CAN1/CAN2 过滤器，启动总线，使能接收中断
 *
 *          3. BMI088_init()
 *             → IMU 上电初始化，软复位，Chip ID 验证，寄存器配置
 *             → 必须在 Gimbal_Init 之前，因为 Gimbal_Init 要读取 IMU 数据
 *
 *          ─────────────────────────────────────────────────────────────────
 *          阶段二：阻塞等待裁判系统数据就绪
 *          ─────────────────────────────────────────────────────────────────
 *          4. while (g_rsi_robot_status.robot_id == 0) {;}
 *             → 等待 RSI UART 回调解析到第一帧裁判系统数据
 *             → 保证后续 Init 能读到有效的热量上限和功率限制
 *             → 阻塞期间 CAN 接收中断正常工作（电机数据已在更新）
 *
 *          ─────────────────────────────────────────────────────────────────
 *          阶段三：控制层 Init（有严格顺序约束）
 *          ─────────────────────────────────────────────────────────────────
 *          5. FrictionWheel_Init()
 *             → 必须在 Shooter_Init 之前（Shooter_Init 内调用 FrictionWheel_SetIdle）
 *             → 启动 TIM8 PWM 通道，输出待机脉宽完成电调握手
 *
 *          6. Gimbal_Init()
 *             → 内部调用 GyroBias_StaticCalib()，阻塞约 2 秒
 *             → 必须在定时器中断（HAL_TIM_Base_Start_IT）之前！
 *               否则中断会在标定完成前开始调用 Gimbal_Control，读到未初始化的 bias
 *             → 以编码器当前值为初始 Pitch 目标，防止上电瞬间突变
 *
 *          7. Chassis_Init()
 *             → 初始化四路速度 PID 和电流低通滤波器
 *             → 无严格顺序约束，放在 Gimbal_Init 后即可
 *
 *          8. Shooter_Init()
 *             → 内部调用 HeatControl_Init，此时裁判系统数据已就绪（步骤4保证）
 *             → HeatControl_Init 会从 g_rsi_robot_status 读取初始热量上限
 *             → 必须在 FrictionWheel_Init 之后
 *
 *          ─────────────────────────────────────────────────────────────────
 *          阶段四：启动控制中断（最后一步！）
 *          ─────────────────────────────────────────────────────────────────
 *          9. HAL_TIM_Base_Start_IT(&htim1)
 *             → 所有 Init 完成后才允许启动控制循环
 *             → 中断一启动，Gimbal_Control/Chassis_Control/Shooter_Control 开始运行
 *             → 这是整个初始化序列中最后一个调用
 *
 *          ─────────────────────────────────────────────────────────────────
 *          阶段五：应用层（中断启动后可调用）
 *          ─────────────────────────────────────────────────────────────────
 *          10. UI_Init()
 *              → 向裁判系统发送初始 UI 元素（瞄准点等）
 *              → 在中断启动后调用，此时裁判系统连接已确认（robot_id != 0）
 *
 * @version 1.0
 * @date    2026-4-6
 * @author  MOS
 ******************************************************************************
 */

#include "app.h"

/* ── BSP 层 ── */
#include "bsp.h"            /* BSP_Init：UART DMA 启动           */
#include "filter.h"         /* can_filter_init：CAN 过滤器与启动  */
#include "BMI088driver.h"   /* BMI088_init：IMU 硬件初始化        */

/* ── 设备驱动层 ── */
#include "rsi_driver.h"     /* g_rsi_robot_status：裁判系统状态   */

/* ── 控制层 ── */
#include "friction_wheel.h" /* FrictionWheel_Init                 */
#include "gimbal.h"         /* Gimbal_Init（含陀螺仪静态标定）    */
#include "chassis.h"        /* Chassis_Init                       */
#include "shooter.h"        /* Shooter_Init（含 HeatControl_Init）*/

/* ── 应用层 ── */
#include "referee_ui.h"     /* UI_Init                            */

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  应用层统一初始化入口
 */
void App_Init(void)
{
    /* ================================================================
     * 阶段一：BSP 层硬件启动
     * ================================================================ */
    /* ① UART DMA 接收启动（必须最先！否则后续等待裁判系统数据的循环永远收不到）*/
    BSP_UART_StartReceive();

    /* ② CAN 过滤器配置 + 总线启动 + 接收中断使能*/
    can_filter_init();

    /* ③ IMU 上电初始化（软复位 + Chip ID 验证 + 寄存器配置）
     *    须在 Gimbal_Init 之前 */
    while (BMI088_init())
	{
		;
	}

    /* ================================================================
     * 阶段二：阻塞等待裁判系统数据就绪（含超时保护）
     * ================================================================
     * 裁判系统上电后会持续发送机器人状态帧（0x0201），
     * RSI UART 回调解析到第一帧后 robot_id 将变为非零值。
     * 阻塞在此处确保：
     *   - HeatControl_Init 能读到正确的热量上限和冷却速率
     *   - Chassis_Control 能读到正确的功率限制
     * 典型等待时间：几百毫秒。
     *
     * 超时策略（REFEREE_WAIT_TIMEOUT_MS）：
     *   超时后各模块 Init 继续执行，使用硬件默认参数：
     *     热量上限  = HC_DEFAULT_Q_MAX_FALLBACK（调试用保守值）
     *     冷却速率  = HC_DEFAULT_Q_COOLDOWN_FALLBACK
     *     功率限制  = Chassis 中读到 0，PID 输出会被 k_limit 缩为 0，
     *                 建议在 Chassis_Control 中对 0 功率限制做保护（TODO）
     *   超时通常意味着调试模式下裁判系统未连接，属于预期情况。
     *   正式比赛时裁判系统必须连接，robot_id 必然在超时前就绪。
     */
    uint32_t wait_start = HAL_GetTick();
    while (g_rsi_robot_status.robot_id == 0U)
    {
        if ((HAL_GetTick() - wait_start) >= REFEREE_WAIT_TIMEOUT_MS)
        {
            /* 超时：裁判系统未连接（调试模式），使用硬件默认参数继续启动。
                * 此时 g_rsi_robot_status 中的热量/功率字段均为 0，
                * HeatControl_Init 和 Chassis_Control 将依赖各自模块的
                * Fallback 默认值（见 heat_control.h / chassis.h 中的注释）。*/
            break;
        }
    }

    /* ================================================================
     * 阶段三：控制层 Init（严格按顺序调用！）
     * ================================================================ */

    /* ④ 摩擦轮 PWM 初始化（必须在 Shooter_Init 之前）
     *    启动 TIM8 CH1/CH2，输出待机脉宽（1000μs）完成电调握手 */
    FrictionWheel_Init();

    /* ⑤ 云台初始化（内含陀螺仪静态标定，阻塞约 2 秒）
     *    !! 必须在 HAL_TIM_Base_Start_IT 之前 !!
     *    标定期间机器人须保持水平静止 */
    Gimbal_Init();

    /* ⑥ 底盘初始化（四路速度 PID + 电流 LPF）*/
    Chassis_Init();

    /* ⑦ 发射机构初始化（含热量控制，此时裁判系统数据已就绪）
     *    内部调用顺序：FrictionWheel_SetIdle → HeatControl_Init → PID_Init */
    Shooter_Init();

    /* ================================================================
     * 阶段四：启动控制中断（最后一步！所有 Init 完成后才允许启动）
     * ================================================================
     * 中断一启动，Gimbal_Control / Chassis_Control / Shooter_Control
     * 将在每个 TIM1 周期（1ms）被调用。
     * 在此之前启动中断会导致控制函数读取未初始化的 PID 状态。
     */
    HAL_TIM_Base_Start_IT(&htim1);
}
