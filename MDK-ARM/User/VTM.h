/**
 ******************************************************************************
 * @file    VTM.h
 * @brief   VTM 图传模块控制头文件（应用层）
 * @note    层级说明：
 *            本文件属于【应用层】，是整个工程最顶层的调度模块之一。
 *            职责：将 VTM 传来的键盘/鼠标/摇杆输入翻译为控制层指令。
 *            依赖链（单向向下）：
 *              VTM → chassis（控制层）
 *              VTM → gimbal（控制层）
 *              VTM → shooter（控制层）
 *              VTM → friction_wheel（控制层）
 *              VTM → bsp_can（BSP 层，安全停机直接发零帧）
 *
 *          VTM 控制模式（由 vtm_ctrl->rc.mode_switch 选择）：
 *            mode_switch = 0 → 键鼠模式
 *            mode_switch = 2 → 摇杆模式（VTM 传 RC 通道值）
 *            mode_switch = 1 → 安全模式（所有电机清零）
 *
 *          底盘旋转控制策略（键鼠模式）：
 *            Q 按下    → 底盘逆时针旋转（速度线性增长到 +MAX_ROTATIONAL_SPEED）
 *            E 按下    → 底盘顺时针旋转（速度线性增长到 -MAX_ROTATIONAL_SPEED）
 *            E 优先于 Q：Q 已按下后再按 E，速度从当前值线性过渡到 -MAX
 *            均未按下  → 底盘随云台偏转角随动（Chassis-Follows-Gimbal）
 *
 * @version 1.0
 * @date    2026-4-6
 * @author  MOS
 ******************************************************************************
 */

#ifndef VTM_H
#define VTM_H

#include "gimbal.h"
#include "chassis.h"
#include "shooter.h"
#include "vtm_driver.h"  /* VTM_ctrl_t 定义 */

/* ========================================================================== */
/*                          底盘速度参数                                        */
/* ========================================================================== */

/** 底盘最大平移速度（与 chassis.h 中 CHASSIS_MAX_TRANSLATIONAL_SPEED 一致）*/
#define VTM_MAX_TRANSLATIONAL_SPEED     500.0f

/** 每帧平移速度增量（@ 1kHz → 5 帧内达到满速，约 5ms 响应）*/
#define VTM_TRANSLATIONAL_ACCEL         100.0f

/* ========================================================================== */
/*                          底盘旋转参数（Q/E 键控制）                          */
/* ========================================================================== */

/** Q/E 旋转最大速度（RPM）*/
#define VTM_MAX_ROTATIONAL_SPEED        300.0f

/**
 * @brief  Q/E 旋转每帧加速量
 * @note   @ 1kHz 调用频率：300 / 100.0 = 3 帧达到最大速度（约 3ms）
 *         若加速过快导致操控不稳，可调小此值。TODO: 根据实际手感调整。
 */
#define VTM_ROTATION_ACCEL              100.0f

/* ========================================================================== */
/*                          底盘随动参数                                        */
/* ========================================================================== */

/**
 * @brief  Yaw 角死区（rad）
 * @note   云台偏转角绝对值小于此值时认为已对齐，旋转速度归零，
 *         避免微小误差引起持续抖动。
 */
#define VTM_YAW_DEADBAND                0.0003f

/**
 * @brief  底盘随动增益（rad → 旋转速度）
 * @note   rotational_velocity = -yaw_angle × VTM_YAW_FOLLOW_GAIN
 *         增益越大，随动响应越快，但可能引起振荡。TODO: 根据实际调试调整。
 */
#define VTM_YAW_FOLLOW_GAIN             200.0f

/* ========================================================================== */
/*                          云台输入灵敏度                                      */
/* ========================================================================== */

/** 鼠标输入灵敏度（鼠标轴值 → 云台角速度 rad/s）TODO: 根据手感调整 */
#define VTM_MOUSE_GIMBAL_SENSITIVITY    0.05f

/** 摇杆通道灵敏度（通道偏差量 → 云台角速度 rad/s）*/
#define VTM_RC_GIMBAL_SENSITIVITY       0.01f

/** 摇杆通道偏差中值 */
#define VTM_RC_CH_CENTER                1024.0f

/** 摇杆通道缩放比（RC 模式下平移速度缩放）*/
#define VTM_RC_SPEED_SCALE              2.0f

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  VTM 图传模块应用控制函数（每帧调用）
 * @note   根据 mode_switch 状态分发控制指令。
 *         须以固定频率（1kHz）在定时器中断中调用。
 * @param  vtm_ctrl  VTM 控制数据结构指针（由 vtm_driver 解析后传入）
 * @retval 无
 */
void VTM_Control(VTM_ctrl_t *vtm_ctrl);

#endif /* VTM_H */
