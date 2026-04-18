/**
 ******************************************************************************
 * @file    gimbal.c
 * @brief   云台控制实现（控制层）
 * @note    职责边界：
 *            【可以调用】设备驱动层（BMI088, motor_driver）、
 *                        算法层（PID, 角度数学）、BSP 层（CAN TX）
 *            【禁止】直接操作寄存器 / HAL 外设，不调用应用层
 *
 * @version 1.1
 * @date    2026-4-5
 * @author  MOS
 ******************************************************************************
 */

#include "gimbal.h"

/* ========================================================================== */
/*                          模块私有状态                                        */
/* ========================================================================== */

static Gimbal_Axis_t s_yaw_axis;           /**< Yaw  轴控制状态（含双环 PID）*/
static Gimbal_Axis_t s_pitch_axis;         /**< Pitch 轴控制状态（含双环 PID）*/
static float         s_yaw_angle_integral; /**< Yaw 角度（相对于大地坐标系）积分量（rad）*/
static GyroBias_t    s_gyro_bias;          /**< 陀螺仪零偏状态（StaticCalib 后持续动态更新）*/

/* ========================================================================== */
/*                              私有函数声明                                    */
/* ========================================================================== */

static float Gimbal_LoopAngle(float angle);
static float Gimbal_ShortestError(float target, float measure);
static float Gimbal_GetPitchAngle(void);

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  云台模块初始化
 */
void Gimbal_Init(void)
{
    /* Step 1：陀螺仪上电静态标定（阻塞 ~2s，须在定时器启动前调用）
     *         机器人在此期间必须保持水平静止 */
    GyroBias_StaticCalib(&s_gyro_bias);

    /* Step 2：初始化 Yaw 角度积分量 */
    s_yaw_angle_integral = 0.0f;

    /* Step 3：初始化 Pitch 轴串级 PID */
    PID_Init(&s_pitch_axis.pid_angle,
             PITCH_ANGLE_KP, PITCH_ANGLE_KI, PITCH_ANGLE_KD,
             PITCH_ANGLE_IOUT_MAX, PITCH_ANGLE_OUT_MAX, PITCH_ANGLE_IGAP);

    PID_Init(&s_pitch_axis.pid_omega,
             PITCH_OMEGA_KP, PITCH_OMEGA_KI, PITCH_OMEGA_KD,
             PITCH_OMEGA_IOUT_MAX, PITCH_OMEGA_OUT_MAX, PITCH_OMEGA_IGAP);

    /* Step 4：初始化 Yaw 轴串级 PID */
    PID_Init(&s_yaw_axis.pid_angle,
             YAW_ANGLE_KP, YAW_ANGLE_KI, YAW_ANGLE_KD,
             YAW_ANGLE_IOUT_MAX, YAW_ANGLE_OUT_MAX, YAW_ANGLE_IGAP);

    PID_Init(&s_yaw_axis.pid_omega,
             YAW_OMEGA_KP, YAW_OMEGA_KI, YAW_OMEGA_KD,
             YAW_OMEGA_IOUT_MAX, YAW_OMEGA_OUT_MAX, YAW_OMEGA_IGAP);

    s_yaw_axis.target_angle = 0.0f;
}

/**
 * @brief  云台控制主函数（每帧调用）
 */
void Gimbal_Control(float yaw_speed, float pitch_speed)
{
    /* ── Step 1：读取 IMU 并做零偏补偿 ── */
    float gyro[3], accel[3], temp;
    BMI088_read(gyro, accel, &temp);
    GyroBias_Correct(&s_gyro_bias, gyro, accel); 

    /* 提取本轴角速度*/
    float pitch_omega = gyro[GIMBAL_GYRO_PITCH_AXIS]; /* rad/s */
    float yaw_omega   = gyro[GIMBAL_GYRO_YAW_AXIS];   /* rad/s */

    /* ── Step 2：Yaw 角度积分 + 目标角度更新 ── */
    s_yaw_angle_integral += yaw_omega * GIMBAL_CONTROL_DT;
    s_yaw_angle_integral  = Gimbal_LoopAngle(s_yaw_angle_integral);

    s_pitch_axis.target_angle += pitch_speed * GIMBAL_CONTROL_DT;
    s_yaw_axis.target_angle   += yaw_speed   * GIMBAL_CONTROL_DT;

    /* Pitch 机械限位保护（在角度环之前截断目标值）*/
    if (s_pitch_axis.target_angle > PITCH_ANGLE_MAX_RAD)
        s_pitch_axis.target_angle = PITCH_ANGLE_MAX_RAD;
    if (s_pitch_axis.target_angle < PITCH_ANGLE_MIN_RAD)
        s_pitch_axis.target_angle = PITCH_ANGLE_MIN_RAD;

    /* Yaw 目标角度归一化到 [0, 2π) */
    s_yaw_axis.target_angle = Gimbal_LoopAngle(s_yaw_axis.target_angle);

    /* ── Step 3：角度环（外环）计算 ── */

    /*
     * 角度环技巧：将最短路径误差"折叠"为普通 PID 的 target-measure：
     *   virtual_measure = actual_measure + shortest_error(target, actual_measure)
     *   PID_Calculate(pid, virtual_measure, actual_measure)
     *     → error = virtual_measure - actual_measure = shortest_error 
     */
    float pitch_angle_now = Gimbal_GetPitchAngle();

    float pitch_virtual_measure = pitch_angle_now
        + Gimbal_ShortestError(s_pitch_axis.target_angle, pitch_angle_now);
    float target_omega_pitch = PID_Calculate(
        &s_pitch_axis.pid_angle, pitch_virtual_measure, pitch_angle_now);

    float yaw_virtual_measure = s_yaw_angle_integral
        + Gimbal_ShortestError(s_yaw_axis.target_angle, s_yaw_angle_integral);
    float target_omega_yaw = PID_Calculate(
        &s_yaw_axis.pid_angle, yaw_virtual_measure, s_yaw_angle_integral);

    /* Pitch 编码器限位时截断角速度指令（防止顶死）*/
    uint16_t pitch_encoder = g_motor_6020[0].rotor_mechanical_angle;
    if ((pitch_encoder >= PITCH_MAX_ENCODER && target_omega_pitch > 0.0f) ||
        (pitch_encoder <= PITCH_MIN_ENCODER && target_omega_pitch < 0.0f))
    {
        target_omega_pitch = 0.0f;
    }

    /* ── Step 4：速度环（内环）计算 + 死区处理 ── */

    /*
     * 死区处理：当角速度误差极小时，用实际角速度代替目标值。
     * 本质上是在速度误差已足够小时关闭 PID 积分累积，
     * 防止因噪声引起的微小误差持续积分产生输出抖动。
     */
    if (fabsf(target_omega_pitch - pitch_omega) <= GIMBAL_PITCH_OMEGA_DEADBAND)
        target_omega_pitch = pitch_omega;
    if (fabsf(target_omega_yaw - yaw_omega) <= GIMBAL_YAW_OMEGA_DEADBAND)
        target_omega_yaw = yaw_omega;

    float torque_pitch = PID_Calculate(&s_pitch_axis.pid_omega,
                                        target_omega_pitch, pitch_omega)
                        + PITCH_EQUILIBRIUM_TORQUE; /* 重力补偿前馈 */
    float torque_yaw   = PID_Calculate(&s_yaw_axis.pid_omega,
                                        target_omega_yaw, yaw_omega);

    /* ── Step 5：打包 CAN 帧并发送 ── */
    uint8_t can_buf[8] = {0U};
    int16_t out_pitch  = (int16_t)torque_pitch;
    int16_t out_yaw    = (int16_t)torque_yaw;

    can_buf[0] = (uint8_t)(out_pitch >> 8);
    can_buf[1] = (uint8_t)(out_pitch);
    can_buf[2] = (uint8_t)(out_yaw >> 8);
    can_buf[3] = (uint8_t)(out_yaw);
    /* can_buf[4~7] 保持 0，对应 ID 3/4 的 6020 电机（预留）*/

    BSP_CAN_TxData(GIMBAL_CAN_HANDLE, GM6020_CTRL_ID_GROUP1, can_buf);
}

/* ========================================================================== */
/*                              私有函数实现                                    */
/* ========================================================================== */

/**
 * @brief  将角度归一化到 [0, 2π)
 * @note   使用 fmodf 替代 while 循环，在实时控制中更安全（无不定次数循环）。
 *         fmodf 可能返回负值（当 angle < 0 时），因此需要条件补偿。
 */
static float Gimbal_LoopAngle(float angle)
{
    angle = fmodf(angle, GIMBAL_2PI);
    if (angle < 0.0f)
        angle += GIMBAL_2PI;
    return angle;
}

/**
 * @brief  计算从 measure 到 target 的最短路径误差，结果在 (-π, π]
 * @note   用于角度环的环形空间误差计算，避免跨越 0/2π 边界时的跳变。
 *         实现：先平移到 [0, 2π)，再减 π 映射到 (-π, π]。
 */
static float Gimbal_ShortestError(float target, float measure)
{
    float delta = fmodf(target - measure + GIMBAL_PI, GIMBAL_2PI);
    if (delta < 0.0f)
        delta += GIMBAL_2PI;
    return delta - GIMBAL_PI;
}

/**
 * @brief  从 6020 编码器值计算 Pitch 当前角度（rad）
 * @note   编码器值加偏移量后对 8192 取模，映射到 [0, 2π)。
 *         PITCH_ENCODER_OFFSET 需根据实际安装位置标定（TODO）。
 */
static float Gimbal_GetPitchAngle(void)
{
    uint16_t raw_encoder  = g_motor_6020[0].rotor_mechanical_angle;
    uint16_t comp_encoder = (uint16_t)((raw_encoder + PITCH_ENCODER_OFFSET)
                             % PITCH_ENCODER_RANGE);
    return ((float)comp_encoder / (float)PITCH_ENCODER_RANGE) * GIMBAL_2PI;
}

/* ========================================================================== */
/*                          公开辅助函数实现                                    */
/* ========================================================================== */
 
/**
 * @brief  获取 Yaw 轴相对底盘的偏转角
 */
float Gimbal_GetYawRelativeAngle(void)
{
    uint16_t raw_encoder  = g_motor_6020[1].rotor_mechanical_angle;
 
    /* 减去中立值后取模，将中立位置归零 */
    uint16_t comp_encoder = (uint16_t)((raw_encoder
                             + GIMBAL_ENCODER_RANGE / 2
                             - GIMBAL_YAW_MIDDLE_ENCODER)
                             % GIMBAL_ENCODER_RANGE);

    /* 映射 [0, 8191] → (-π, π]：中立位置 = 0，CCW 为正 */
    return ((float)comp_encoder / (float)GIMBAL_ENCODER_RANGE)
           * GIMBAL_2PI - GIMBAL_PI;
}
