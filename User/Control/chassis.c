/**
 ******************************************************************************
 * @file    chassis.c
 * @brief   底盘控制实现（控制层）
 * @note    职责边界：
 *            【可以调用】pid / lpf（算法层）、motor_driver / rsi_driver（设备驱动层）、
 *                        bsp_can（BSP 层）
 *            【禁止】直接调用 HAL_xxx 函数或访问 GPIOx / CANx 寄存器
 *
 * @version 1.0
 * @date    2026-4-6
 * @author  MOS
 ******************************************************************************
 */

#include "chassis.h"

/* ========================================================================== */
/*                          模块私有状态                                        */
/* ========================================================================== */

static PID_t         s_pid_speed[CHASSIS_MOTOR_COUNT];   /**< 四路速度环 PID  */
static LowPassFilter_t s_lpf_torque[CHASSIS_MOTOR_COUNT]; /**< 四路电流低通滤波 */

/* ========================================================================== */
/*                          私有函数：CAN 发送封装                              */
/* ========================================================================== */

/**
 * @brief  打包并发送四路 3508 转矩电流 CAN 帧
 * @note   帧格式（大端）：
 *           Byte 0-1: 电机 1（左前）电流
 *           Byte 2-3: 电机 2（左后）电流
 *           Byte 4-5: 电机 3（右后）电流
 *           Byte 6-7: 电机 4（右前）电流
 * @param  motor_out  四路输出电流数组（有符号，正值正转）
 */
static void Chassis_SendMotorCurrent(const int16_t motor_out[CHASSIS_MOTOR_COUNT])
{
    uint8_t buf[8] = {0U};
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
    {
        buf[i * 2U]      = (uint8_t)((uint16_t)motor_out[i] >> 8U);
        buf[i * 2U + 1U] = (uint8_t)((uint16_t)motor_out[i] & 0xFFU);
    }
    BSP_CAN_TxData(CHASSIS_CAN_HANDLE, M2006_M3508_CTRL_ID_GROUP1, buf);
}

/* ========================================================================== */
/*                          私有函数：功率控制                                  */
/* ========================================================================== */

/**
 * @brief  计算功率限制缩放系数 k
 * @note   功率估算：P ≈ Σ|I_filtered(A)| × V_nominal
 *           I_filtered = LPF(actual_quadrature_current) / CURRENT_TO_AMP
 *
 *         控制策略：
 *           P ≤ limit - POWER_MARGIN      → k = 1.0（不限制）
 *           P > limit - POWER_MARGIN      → k = (1/P) × buffer_energy × COEFF
 *                                              （缓冲能量越充足，允许输出越高）
 *         k 由 static 变量保持，具有隐式的一阶低通效果（每帧更新）。
 *
 * @param  limit_power  裁判系统下发的当前功率上限（W）
 * @retval 功率缩放系数 k ∈ (0, 1]
 */
float monitor_k = 0.0f;
static float Chassis_CalcPowerLimit(float limit_power)
{
    static float s_k = 1.0f; /* 保持上一帧系数，具有平滑效果 */

    /* 估算总电流幅值（单位：A）*/
    float total_current_a = 0.0f;
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
    {
        float filtered = LPF_Update(&s_lpf_torque[i],
                                    (float)g_motor_3508[i].actual_quadrature_current);
        total_current_a += fabsf(filtered);
    }

    /* 估算当前底盘功率（P ≈ I_total × V，粗略估算）*/
    float power_estimate = (total_current_a / CHASSIS_CURRENT_TO_AMP)
                           * CHASSIS_NOMINAL_VOLTAGE_V;

    if (power_estimate > (limit_power - CHASSIS_POWER_MARGIN_W))
    {
        /* 超过裕量线：利用缓冲能量动态缩放，缓冲能量越多允许输出越高 */
        s_k = (1.0f / power_estimate)
              * (float)g_rsi_power_heat.buffer_energy
              * CHASSIS_BUFFER_ENERGY_COEFF;
    }
    else
    {
        /* 功率充裕：全功率输出 */
        s_k = 1.0f;
    }

    monitor_k = s_k;

    return s_k;
}

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  底盘控制模块初始化
 */
void Chassis_Init(void)
{
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
    {
        PID_Init(&s_pid_speed[i],
                 CHASSIS_SPEED_KP, CHASSIS_SPEED_KI, CHASSIS_SPEED_KD,
                 CHASSIS_SPEED_IOUT_MAX, CHASSIS_SPEED_OUT_MAX, CHASSIS_SPEED_IGAP);

        LPF_Init(&s_lpf_torque[i], CHASSIS_LPF_DT, CHASSIS_LPF_FC);
    }
}

/**
 * @brief  底盘控制主函数
 */
void Chassis_Control(float v_x, float v_y, float v_w, float yaw_gimbal_to_chassis)
{
    /* ── Step 1：麦轮运动学逆解 ── */

    /*
     * 将速度指令从云台坐标系旋转到底盘坐标系，再做麦轮分解。
     * 旋转角 = yaw_gimbal_to_chassis + π/4（麦轮 45° 安装补偿）
     *
     * 提前计算 cos/sin 避免四次重复三角函数调用。
     */
    float theta     = yaw_gimbal_to_chassis + CHASSIS_WHEEL_ANGLE_RAD;
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);

    /* 中间量：cos/sin 旋转后的合速度分量 */
    float v_diag_a =  cos_theta * v_x - sin_theta * v_y; /* 对角线 1/3 方向 */
    float v_diag_b =  sin_theta * v_x + cos_theta * v_y; /* 对角线 2/4 方向 */

    /*
     * 各轮目标转速（已折算到电机轴输出转速，单位与 PID 设定值一致）
     * 轮序：[0]=左前, [1]=左后, [2]=右后, [3]=右前
     */
    float speed_target[CHASSIS_MOTOR_COUNT];
    speed_target[0] =  v_diag_a + v_w;
    speed_target[1] =  v_diag_b + v_w;
    speed_target[2] = -v_diag_a + v_w;
    speed_target[3] = -v_diag_b + v_w;

    /* ── Step 2：功率限制系数 ── */
    float limit_power = 0.0f;
    if (g_rsi_robot_status.robot_id == 0U)
        limit_power = CHASSIS_FALLBACK_LIMIT_POWER;
    else
        limit_power = (float)g_rsi_robot_status.chassis_power_limit;
    float k_limit = Chassis_CalcPowerLimit(limit_power);

    /* ── Step 3：速度环 PID 计算（含功率缩放）── */
    int16_t motor_out[CHASSIS_MOTOR_COUNT];
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
    {
        /* 编码器转速 → 轮端转速（除以减速比）*/
        float speed_measure = (float)g_motor_3508[i].rotor_speed / CHASSIS_REDUCTION_RATIO;

        float pid_out = PID_Calculate(&s_pid_speed[i], speed_target[i], speed_measure);

        /* 乘以功率缩放系数，限制在额定功率内 */
        motor_out[i] = (int16_t)(pid_out * k_limit);
    }

    /* ── Step 4：发送转矩指令 ── */
    Chassis_SendMotorCurrent(motor_out);
}
