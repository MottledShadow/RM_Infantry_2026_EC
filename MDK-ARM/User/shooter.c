/**
 ******************************************************************************
 * @file    shooter.c
 * @brief   发射机构控制实现（控制层）
 * @note    职责边界：
 *            【可以调用】heat_control（同层）、friction_wheel（同层）、
 *                        pid（算法层）、motor_driver（设备驱动层）、
 *                        bsp_can（BSP 层）
 *            【禁止】直接调用 HAL_xxx 函数或访问 GPIOx 寄存器
 *
 *          内部模块划分：
 *            ├── M2006 CAN 发送封装（m2006_send_torque）
 *            ├── 编码器累积追踪（encoder_*）
 *            ├── 连发控制（launch_omega）
 *            ├── 单发状态机（launch_once_*）
 *            └── 卡弹检测与恢复状态机（jam_*）
 *
 * @version 1.0
 * @date    2026-4-5
 * @author  黄仲华
 ******************************************************************************
 */

#include "shooter.h"

/* ========================================================================== */
/*                          工具宏（内部使用）                                  */
/* ========================================================================== */

/** 重置 PID err[] 数组（3个 float 清零）*/
#define RESET_PID(e)        memset((e), 0, 3 * sizeof(float))

/** 对称限幅 */
#define CLAMPF(x, lo, hi)   ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

/* ========================================================================== */
/*                          模块私有状态                                        */
/* ========================================================================== */

HeatControl_t g_heat_ctrl; /**< 热量控制实例，对外只读（shooter.h 中声明 extern）*/
HeatControl_Config_t g_heat_ctrl_cfg; /**< 热量控制配置实例（赛前由 Config 层写入参数）*/

static Shooter_t s_shooter; /**< 拨弹盘串级 PID 实例（外部不可见）*/

/* ── 发射机构整体状态 ── */
static uint8_t s_shooter_enabled       = 0U;
static uint8_t s_safety_btn_last       = 0U;
static uint8_t s_fire_btn_last         = 0U;
static uint8_t s_once_btn_was_pressed  = 0U;
static uint8_t s_single_fire_triggered = 0U;

/* ── 编码器累积追踪 ── */
static float    s_total_encoder       = 0.0f;
static uint16_t s_encoder_last        = 0U;
static uint8_t  s_encoder_initialized = 0U;

/* ── 单发状态机 ── */
static uint8_t s_launch_once_state          = 0U;
static float   s_launch_once_target_encoder = 0.0f;
static float   s_angle_error_once[3]        = {0.0f};
static float   s_error_2006_once[3]         = {0.0f};

/* ── 卡弹恢复状态机 ── */
static uint8_t  s_jam_detected               = 0U;
static uint16_t s_jam_counter                = 0U;
static uint8_t  s_jam_recovery_state         = 0U;
static float    s_jam_recovery_target_encoder = 0.0f;
static float    s_angle_error_jam[3]          = {0.0f};
static float    s_error_2006_jam[3]           = {0.0f};

/* ========================================================================== */
/*                          私有函数：CAN 发送封装                              */
/* ========================================================================== */

/**
 * @brief  向 M2006 发送转矩电流指令
 * @note   封装重复的 CAN 帧组装逻辑，大端格式，Byte 0-1 为转矩值，Byte 2-7 填 0。
 *         使用 hcan2（云台 + 拨弹盘共用总线）。
 * @param  torque_val  目标转矩电流值（有符号，正值正转）
 */
static void M2006_SendTorque(int16_t torque_val)
{
    uint8_t buf[8] = {0U};
    buf[0] = (uint8_t)((uint16_t)torque_val >> 8);
    buf[1] = (uint8_t)((uint16_t)torque_val & 0xFFU);
    BSP_CAN_TxData(SHOOTER_CAN_HANDLE, M2006_M3508_CTRL_ID_GROUP1, buf);
}

/* ========================================================================== */
/*                          私有函数：编码器累积追踪                            */
/* ========================================================================== */

/**
 * @brief  计算两帧之间的编码器增量（处理 0/8191 边界回绕）
 * @param  cur   当前编码器值
 * @param  last  上一帧编码器值
 * @retval 有符号增量（-4096 ~ +4096）
 */
static int32_t Encoder_Delta(uint16_t cur, uint16_t last)
{
    int32_t d = (int32_t)cur - (int32_t)last;
    if      (d < -(int32_t)(SHOOTER_ENCODER_MAX / 2U)) d += (int32_t)SHOOTER_ENCODER_MAX;
    else if (d >  (int32_t)(SHOOTER_ENCODER_MAX / 2U)) d -= (int32_t)SHOOTER_ENCODER_MAX;
    return d;
}

/**
 * @brief  读取 M2006 编码器并累加到 s_total_encoder
 * @note   首次调用时完成初始化（记录基准值），不产生增量。
 *         须在每个使用 s_total_encoder 的状态机 tick 开头调用。
 */
static void Encoder_Update(void)
{
    uint16_t cur = g_motor_2006.rotor_mechanical_angle;
    if (!s_encoder_initialized)
    {
        s_encoder_last        = cur;
        s_encoder_initialized = 1U;
        return;
    }
    s_total_encoder += (float)Encoder_Delta(cur, s_encoder_last);
    s_encoder_last   = cur;
}

/**
 * @brief  重置编码器累积量（切换控制模式时调用）
 */
static void Encoder_Reset(void)
{
    s_total_encoder       = 0.0f;
    s_encoder_initialized = 0U;
}

/* ========================================================================== */
/*                          私有函数：连发控制                                  */
/* ========================================================================== */

/**
 * @brief  连发模式：直接指定目标转速，走速度环
 * @param  target_omega  目标转速（RPM），通常由 HeatControl_GetTargetOmega() 提供
 */
static void Launch_Omega(float target_omega)
{
    int16_t torque = (int16_t)PID_Calculate(
        &s_shooter.pid_omega, target_omega, (float)g_motor_2006.rotor_speed);
    M2006_SendTorque(torque);
}

/* ========================================================================== */
/*                          私有函数：单发状态机                                */
/* ========================================================================== */

/**
 * @brief  复位单发状态机至空闲状态
 */
static void LaunchOnce_Reset(void)
{
    s_launch_once_state = 0U;
    RESET_PID(s_angle_error_once);
    RESET_PID(s_error_2006_once);
}

/**
 * @brief  查询单发状态机是否正在执行中
 * @retval 1: 执行中，0: 空闲
 */
static uint8_t LaunchOnce_IsBusy(void)
{
    return s_launch_once_state != 0U;
}

/**
 * @brief  单发状态机 tick（每帧调用）
 * @note   三段式状态机：
 *           state 0 → 初始化目标编码器 = 当前值 + 一颗弹行程，进入 state 1
 *           state 1 → 位置环 + 速度环串级控制，误差小且转速低时进入 state 2
 *           state 2 → 主动制动（输出 0），转速足够低后回到 state 0（完成）
 *
 *         state 1 中"到位"判断策略：
 *           ① 误差 ≥ NEAR_THRESHOLD：正常 PID 推进
 *           ② 误差 < NEAR_THRESHOLD：清零误差防积分累积，等待转速下降
 *           ③ 误差 < NEAR_THRESHOLD AND 转速 < STOP_OMEGA_THRESHOLD → 进入制动
 */
static void LaunchOnce_Tick(void)
{
    Encoder_Update();

    switch (s_launch_once_state)
    {
        case 0U:
        {
            /* 初始化目标位置，启动控制 */
            s_launch_once_target_encoder = s_total_encoder + SHOOTER_ENCODER_PER_BULLET;
            RESET_PID(s_angle_error_once);
            RESET_PID(s_error_2006_once);
            s_launch_once_state = 1U;
            break;
        }
        case 1U:
        {
            s_angle_error_once[0] = s_launch_once_target_encoder - s_total_encoder;

            float omega_target = 0.0f;
            if (fabsf(s_angle_error_once[0]) >= SHOOTER_NEAR_THRESHOLD)
            {
                s_shooter.target_angle = s_total_encoder + s_angle_error_once[0];
                omega_target = PID_Calculate(
                    &s_shooter.pid_angle, s_shooter.target_angle, s_total_encoder);
                omega_target = CLAMPF(omega_target, -SHOOTER_OMEGA_CLAMP, SHOOTER_OMEGA_CLAMP);
            }
            else
            {
                s_angle_error_once[0] = 0.0f; /* 到位后清零，防止积分继续累积 */
            }

            /* 到位且停稳：进入主动制动阶段 */
            if (fabsf(s_angle_error_once[0]) < SHOOTER_NEAR_THRESHOLD &&
                fabsf((float)g_motor_2006.rotor_speed) < SHOOTER_STOP_OMEGA_THRESHOLD)
            {
                s_launch_once_state = 2U;
            }

            s_error_2006_once[0] = omega_target - (float)g_motor_2006.rotor_speed;
            int16_t t = (int16_t)PID_Calculate(
                &s_shooter.pid_omega, omega_target, (float)g_motor_2006.rotor_speed);
            M2006_SendTorque(t);
            break;
        }
        case 2U:
        {
            /* 主动制动：输出 0，等待完全静止 */
            M2006_SendTorque(0);
            if (fabsf((float)g_motor_2006.rotor_speed) < SHOOTER_IDLE_OMEGA_THRESHOLD)
                s_launch_once_state = 0U;
            break;
        }
        default:
        {
            s_launch_once_state = 0U;
            break;
        }
    }
}

/* ========================================================================== */
/*                          私有函数：卡弹检测与恢复                            */
/* ========================================================================== */

/**
 * @brief  重置卡弹状态机至初始状态
 */
static void Jam_Reset(void)
{
    s_jam_detected       = 0U;
    s_jam_counter        = 0U;
    s_jam_recovery_state = 0U;
    RESET_PID(s_angle_error_jam);
    RESET_PID(s_error_2006_jam);
}

/**
 * @brief  卡弹检测：低速 + 高转矩 持续多帧判定为卡弹
 * @note   仅在发射机构启用时检测。
 *         两个条件同时满足超过 JAM_COUNT_THRESHOLD 帧才判定，
 *         任一条件不满足则计数器清零（要求连续满足）。
 * @retval 1: 检测到卡弹，0: 正常
 */
static uint8_t Jam_Detect(void)
{
    if (s_shooter_enabled &&
        fabsf((float)g_motor_2006.rotor_speed)              < (float)SHOOTER_JAM_OMEGA_THRESHOLD &&
        fabsf((float)g_motor_2006.actual_quadrature_current) > (float)SHOOTER_JAM_TORQUE_THRESHOLD)
    {
        if (++s_jam_counter >= SHOOTER_JAM_COUNT_THRESHOLD)
            return 1U;
    }
    else
    {
        s_jam_counter = 0U;
    }
    return 0U;
}

/**
 * @brief  触发卡弹恢复：强制中断单发状态机，启动反转恢复
 * @note   反转目标 = 当前编码器 - 半颗弹行程，以退出卡弹位置。
 */
static void Jam_StartRecovery(void)
{
    LaunchOnce_Reset();
    Encoder_Update();
    s_jam_recovery_target_encoder = s_total_encoder - SHOOTER_JAM_REVERSE_DISTANCE;
    s_jam_detected       = 1U;
    s_jam_counter        = 0U;
    s_jam_recovery_state = 1U;
    RESET_PID(s_angle_error_jam);
    RESET_PID(s_error_2006_jam);
}

/**
 * @brief  查询卡弹恢复是否进行中
 * @retval 1: 恢复中，0: 已完成或未触发
 */
static uint8_t Jam_IsRecovering(void)
{
    return s_jam_recovery_state != 0U;
}

/**
 * @brief  卡弹恢复状态机 tick（每帧调用）
 * @note   两段式：
 *           state 1 → 反转半颗弹距离（位置环 + 速度环）
 *           state 2 → 制动，完成后清除所有卡弹标志
 */
static void Jam_RecoverTick(void)
{
    Encoder_Update();

    switch (s_jam_recovery_state)
    {
        case 1U:
        {
            s_angle_error_jam[0] = s_jam_recovery_target_encoder - s_total_encoder;

            float omega_target = 0.0f;
            if (fabsf(s_angle_error_jam[0]) >= SHOOTER_NEAR_THRESHOLD)
            {
                s_shooter.target_angle = s_total_encoder + s_angle_error_jam[0];
                omega_target = PID_Calculate(
                    &s_shooter.pid_angle, s_shooter.target_angle, s_total_encoder);
                omega_target = CLAMPF(omega_target, -SHOOTER_OMEGA_CLAMP, SHOOTER_OMEGA_CLAMP);
            }

            if (fabsf(s_angle_error_jam[0])          < SHOOTER_NEAR_THRESHOLD &&
                fabsf((float)g_motor_2006.rotor_speed) < SHOOTER_STOP_OMEGA_THRESHOLD)
            {
                s_jam_recovery_state = 2U;
            }

            s_error_2006_jam[0] = omega_target - (float)g_motor_2006.rotor_speed;
            int16_t t = (int16_t)PID_Calculate(
                &s_shooter.pid_omega, omega_target, (float)g_motor_2006.rotor_speed);
            M2006_SendTorque(t);
            break;
        }
        case 2U:
        {
            M2006_SendTorque(0);
            s_jam_recovery_state = 0U;
            s_jam_detected       = 0U;
            break;
        }
        default:
        {
            s_jam_recovery_state = 0U;
            break;
        }
    }
}

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  发射机构初始化
 */
void Shooter_Init(void)
{
    FrictionWheel_SetIdle();
    HeatControl_Init(&g_heat_ctrl, NULL);  /* NULL = 使用 HC_DEFAULT_* 宏中的默认赛规参数 */

    LaunchOnce_Reset();
    Jam_Reset();
    Encoder_Reset();

    s_shooter_enabled       = 0U;
    s_safety_btn_last       = 0U;
    s_fire_btn_last         = 0U;
    s_once_btn_was_pressed  = 0U;
    s_single_fire_triggered = 0U;

    /* 初始化拨弹盘串级 PID */
    PID_Init(&s_shooter.pid_angle,
             SHOOTER_ANGLE_KP, SHOOTER_ANGLE_KI, SHOOTER_ANGLE_KD,
             SHOOTER_ANGLE_IOUT_MAX, SHOOTER_ANGLE_OUT_MAX, SHOOTER_ANGLE_IGAP);

    PID_Init(&s_shooter.pid_omega,
             SHOOTER_OMEGA_KP, SHOOTER_OMEGA_KI, SHOOTER_OMEGA_KD,
             SHOOTER_OMEGA_IOUT_MAX, SHOOTER_OMEGA_OUT_MAX, SHOOTER_OMEGA_IGAP);
}

/**
 * @brief  查询发射机构启用状态
 */
uint8_t Shooter_IsEnabled(void)
{
    return s_shooter_enabled;
}

/**
 * @brief  设置发射机构启用/禁用
 */
void Shooter_SetEnabled(uint8_t enabled)
{
    s_shooter_enabled = enabled;
    if (enabled)
    {
        FrictionWheel_SetReady();
    }
    else
    {
        FrictionWheel_SetIdle();
        LaunchOnce_Reset();
        Jam_Reset();
    }
}

/**
 * @brief  发射机构主控制函数
 */
void Shooter_Control(uint8_t safety_btn, uint8_t fire_btn, uint8_t once_btn)
{
    /* 每帧更新热量冷却衰减与目标转速 */
    HeatControl_Update(&g_heat_ctrl);

    /*
     * do-while(0) 结构实现优先级分支：
     * 任一分支 break 后跳出，避免 goto 同时保持清晰的优先级顺序。
     * 按键历史统一在结构末尾更新，各分支不重复赋值。
     */
    do {
        /* 优先级 1：未启用 → 等待安全键上升沿触发启用 */
        if (!s_shooter_enabled)
        {
            if (safety_btn && !s_safety_btn_last)
            {
                Shooter_SetEnabled(1U);
                FrictionWheel_SetFire();
            }
            break;
        }

        /* 优先级 2：卡弹恢复中 → 执行反转恢复，屏蔽其他指令 */
        if (Jam_IsRecovering())
        {
            Jam_RecoverTick();
            break;
        }

        /* 优先级 3：连发键按住 → 连发控制，同时检测卡弹 */
        if (fire_btn)
        {
            Launch_Omega(HeatControl_GetTargetOmega(&g_heat_ctrl));
            if (Jam_Detect())
                Jam_StartRecovery();
            break;
        }

        /* 优先级 4：连发键松开（下降沿）→ 停转，清单发标志 */
        if (s_fire_btn_last && !fire_btn)
        {
            Launch_Omega(0.0f);
            LaunchOnce_Reset();
            s_single_fire_triggered = 0U;
            break;
        }

        /* 优先级 5a：单发键按住 → 标记"待发"（若此时正在执行单发则忽略本次） */
        if (once_btn)
        {
            s_once_btn_was_pressed = !LaunchOnce_IsBusy();
            break;
        }

        /* 优先级 5b：单发键松开 + 有待发标记 → 触发一次单发 */
        if (s_once_btn_was_pressed && !s_single_fire_triggered)
        {
            s_once_btn_was_pressed = 0U;
            float q_res = g_heat_ctrl.q_max - g_heat_ctrl.q_now;
            if (!LaunchOnce_IsBusy() && q_res >= g_heat_ctrl.q_stop)
            {
                LaunchOnce_Tick();              /* state 0 → 1：初始化目标位置 */
                HeatControl_AddHeat(&g_heat_ctrl); /* 预扣热量，防止下一帧发第二发 */
                s_single_fire_triggered = 1U;
            }
            break;
        }

        /* 单发执行完成：清除触发标志 */
        if (s_single_fire_triggered && !LaunchOnce_IsBusy())
            s_single_fire_triggered = 0U;

        /* 优先级 6：安全键上升沿 → 关闭发射机构 */
        if (safety_btn && !s_safety_btn_last)
        {
            Shooter_SetEnabled(0U);
            break;
        }

        /* 优先级 7：空闲 / 单发执行中 */
        if (LaunchOnce_IsBusy())
        {
            LaunchOnce_Tick();
            if (Jam_Detect())
                Jam_StartRecovery();
        }
        else
        {
            M2006_SendTorque(0); /* 空闲时保持零转矩 */
        }

    } while (0);

    /* 统一更新按键历史（防止各分支遗漏） */
    s_safety_btn_last = safety_btn;
    s_fire_btn_last   = fire_btn;
}
