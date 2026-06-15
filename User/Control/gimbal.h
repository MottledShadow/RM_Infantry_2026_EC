/**
 ******************************************************************************
 * @file    gimbal.h
 * @brief   云台控制头文件（控制层）
 * @note    层级说明：
 *            本文件属于【控制层】，位于设备驱动层之上、应用层之下。
 *            依赖链（单向向下）：
 *              gimbal → pid（算法层）
 *              gimbal → gyro_calibration / motor_driver（设备驱动层）
 *              gimbal → bsp_can（BSP 层）
 *
 *          云台控制结构：
 *            双轴（Pitch / Yaw）各自采用串级 PID：
 *              外环：角度环（rad）  → 输出目标角速度（rad/s）
 *              内环：速度环（rad/s）→ 输出电机电流（目标转矩）
 *            Pitch 轴测量：6020 电机编码器（绝对角度）
 *            Yaw  轴测量：BMI088 陀螺仪 Z 轴（角速度积分）
 *
 * @version 1.1
 * @date    2026-4-5
 * @author  MOS
 ******************************************************************************
 */

#ifndef GIMBAL_H
#define GIMBAL_H

#include <math.h>               /* fabsf, fmodf               */
#include "bsp.h"            /* BSP 层：CAN 发送           */
#include "gyro_calibration.h"   /* 设备驱动层：IMU 零偏补偿   */
#include "BMI088driver.h"       /* 设备驱动层：IMU 原始读取   */
#include "motor_driver.h"       /* 设备驱动层：电机反馈数据   */
#include "pid.h"                /* 算法层 */

/* ========================================================================== */
/*                              数学常量                                        */
/* ========================================================================== */

#ifndef GIMBAL_PI
#define GIMBAL_PI       3.14159265358979323846f
#define GIMBAL_2PI      (2.0f * GIMBAL_PI)
#endif

/* ========================================================================== */
/*                          Pitch 轴硬件参数                                    */
/* ========================================================================== */

/**
 * @brief  Pitch 编码器零偏补偿值
 * @note   6020 编码器范围 [0, 8191]，加上偏移后对准机械零点。
 *         TODO: 根据实际云台安装位置标定此值。
 */
#define PITCH_ENCODER_OFFSET        6692U

/** 编码器量程 */
#define PITCH_ENCODER_RANGE         8192U

/** Pitch 机械限位：编码器上限（对应俯仰最大角度）*/
#define PITCH_MAX_ENCODER           2600U

/** Pitch 机械限位：编码器下限（对应俯仰最小角度）*/
#define PITCH_MIN_ENCODER           1500U

/** Pitch 角度上限（rad），约 48.7°，防止打架 */
#define PITCH_ANGLE_MAX_RAD         0.85f

/** Pitch 角度下限（rad）*/
#define PITCH_ANGLE_MIN_RAD         0.0f

/* ========================================================================== */
/*                          Yaw 轴编码器参数                                    */
/* ========================================================================== */
 
/**
 * @brief  Yaw 轴编码器中立值（电机机械零点对应的原始编码器值）
 * @note   6020 编码器范围 [0, 8191]，此值对应云台正对底盘前方的位置。
 *         TODO: 根据实际云台安装位置标定此值。
 */
#define GIMBAL_YAW_MIDDLE_ENCODER   7505U
 
/** 编码器量程（与 Pitch 共用同一型号电机）*/
#define GIMBAL_ENCODER_RANGE        8192U

/* ========================================================================== */
/*                          控制参数（dt）                                      */
/* ========================================================================== */

/** 控制周期（s），须与定时器中断频率一致 */
#define GIMBAL_CONTROL_DT           0.001f  /* 1ms = 1kHz */

/** 速度环死区阈值：Pitch（rad/s）*/
#define GIMBAL_PITCH_OMEGA_DEADBAND 0.05f

/** 速度环死区阈值：Yaw（rad/s）*/
#define GIMBAL_YAW_OMEGA_DEADBAND   0.1f

/* ========================================================================== */
/*                          Pitch 轴 PID 参数                                  */
/* ========================================================================== */

#define PITCH_ANGLE_KP              20.0f
#define PITCH_ANGLE_KI              0.0f
#define PITCH_ANGLE_KD              0.0f
#define PITCH_ANGLE_IOUT_MAX        5.0f    /**< 角度环积分限幅（rad/s）*/
#define PITCH_ANGLE_OUT_MAX         10.0f   /**< 角度环输出限幅（rad/s）*/
#define PITCH_ANGLE_IGAP            0.0f    /**< 角度环积分分离阈值（0=不分离）*/

#define PITCH_OMEGA_KP              3500.0f
#define PITCH_OMEGA_KI              500.0f
#define PITCH_OMEGA_KD              0.0f
#define PITCH_OMEGA_IOUT_MAX        1000.0f /**< 速度环积分限幅（电流值）*/
#define PITCH_OMEGA_OUT_MAX         16000.0f/**< 速度环输出限幅（电流值）*/
#define PITCH_OMEGA_IGAP            0.2f    /**< 速度环积分分离阈值（rad/s）*/

/** Pitch 轴重力补偿前馈力矩（电流值），用于抵消重力矩 */
#define PITCH_EQUILIBRIUM_TORQUE    4000.0f

/* ========================================================================== */
/*                          Yaw 轴 PID 参数                                    */
/* ========================================================================== */

#define YAW_ANGLE_KP                30.0f
#define YAW_ANGLE_KI                0.0f
#define YAW_ANGLE_KD                0.0f
#define YAW_ANGLE_IOUT_MAX          5.0f
#define YAW_ANGLE_OUT_MAX           10.0f
#define YAW_ANGLE_IGAP              0.0f

#define YAW_OMEGA_KP                6400.0f
#define YAW_OMEGA_KI                50.0f
#define YAW_OMEGA_KD                0.0f
#define YAW_OMEGA_IOUT_MAX          1000.0f
#define YAW_OMEGA_OUT_MAX           16000.0f
#define YAW_OMEGA_IGAP              0.05f

/* ========================================================================== */
/*                              陀螺仪轴映射                                    */
/* ========================================================================== */

/**
 * @brief  BMI088 陀螺仪与云台轴的对应关系
 * @note   取决于 IMU 安装方向，TODO: 根据实际安装校验符号和轴索引。
 */
#define GIMBAL_GYRO_PITCH_AXIS      0   /**< BMI088 gyro[0] 对应 Pitch 轴角速度 */
#define GIMBAL_GYRO_YAW_AXIS        2   /**< BMI088 gyro[2] 对应 Yaw  轴角速度  */

/* ========================================================================== */
/*                              数据结构定义                                    */
/* ========================================================================== */

/**
 * @brief  单轴云台控制状态
 */
typedef struct
{
    float target_angle; /**< 目标角度（rad），由外部指令累积         */
    PID_t pid_angle;    /**< 外环：角度 PID                          */
    PID_t pid_omega;    /**< 内环：角速度 PID                        */
} Gimbal_Axis_t;

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  云台模块初始化
 * @note   完成以下工作：
 *           1. 陀螺仪上电静态零偏标定（阻塞约 2s，机器人须静止）
 *           2. Pitch / Yaw 双轴串级 PID 初始化
 *         须在定时器中断启动（HAL_TIM_Base_Start_IT）之前调用。
 * @retval 无
 */
void Gimbal_Init(void);

/**
 * @brief  获取 Yaw 轴相对底盘的偏转角（rad）
 * @note   基于 6020 Yaw 电机编码器计算，范围 (-π, π]，
 *         0 表示云台正对底盘前方，正值为 CCW（逆时针）。
 *         供底盘控制使用：
 *           1. 作为 Chassis_Control() 的 yaw_gimbal_to_chassis 参数
 *           2. 作为底盘随动控制的反馈输入
 * @retval Yaw 轴相对角度（rad），范围 (-π, π]
 */
float Gimbal_GetYawRelativeAngle(void);

/**
 * @brief  云台控制主函数（每帧调用）
 * @note   须以 1/GIMBAL_CONTROL_DT Hz 固定频率在定时器中断中调用。
 *         执行流程：
 *           1. 读取 BMI088 并进行零偏补偿
 *           2. 积分 Yaw 角度，更新目标角度
 *           3. 角度环（外环）PID 计算
 *           4. 速度环（内环）PID 计算（含死区与限位保护）
 *           5. 打包 CAN 帧发送电流指令
 * @param  yaw_speed    Yaw 轴角速度指令（rad/s），正值为右偏
 * @param  pitch_speed  Pitch 轴角速度指令（rad/s），正值为上仰
 * @retval 无
 */
void Gimbal_Control(float yaw_speed, float pitch_speed);

#endif /* GIMBAL_H */
