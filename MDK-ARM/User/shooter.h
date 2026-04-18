/**
 ******************************************************************************
 * @file    shooter.h
 * @brief   发射机构控制头文件（控制层）
 * @note    层级说明：
 *            本文件属于【控制层】，位于设备驱动层之上、应用层之下。
 *            依赖链（单向向下）：
 *              shooter → heat_control（控制层，热量管理）
 *              shooter → friction_wheel（控制层，摩擦轮）
 *              shooter → pid（算法层）
 *              shooter → motor_driver（设备驱动层，读 g_motor_2006）
 *              shooter → bsp_can（BSP 层，发 M2006 转矩）
 *
 *          控制结构：
 *            拨弹盘 M2006 采用串级 PID：
 *              外环：位置环（编码器累积值）→ 输出目标转速（RPM）
 *              内环：速度环（RPM）          → 输出转矩电流
 *            单发与连发共用同一套 PID 实例，通过状态机分时使用。
 *
 * @version 1.0
 * @date    2026-4-5
 * @author  黄仲华
 ******************************************************************************
 */

#ifndef SHOOTER_H
#define SHOOTER_H

#include <string.h>
#include <math.h>
#include "heat_control.h"
#include "pid.h"
#include "motor_driver.h"    /* 设备驱动层：g_motor_2006 读取         */
#include "friction_wheel.h"  /* 控制层：摩擦轮状态切换                */
#include "bsp.h"         /* BSP 层：CAN 发送（控制层→BSP，方向正确）*/

/* ========================================================================== */
/*                          编码器与机械参数                                    */
/* ========================================================================== */

#define SHOOTER_ENCODER_MAX             8192U       /**< M2006 编码器量程             */
#define SHOOTER_ROTOR_TO_DIAL_RATIO     36U         /**< M2006 减速比（电机转/拨盘转）*/
#define SHOOTER_BULLETS_PER_DIAL        8U          /**< 拨弹盘每转发出的弹数         */

/** 每发弹对应的电机编码器累积量 = 量程 × 减速比 / 每转弹数 */
#define SHOOTER_ENCODER_PER_BULLET  \
    ((float)(SHOOTER_ENCODER_MAX * SHOOTER_ROTOR_TO_DIAL_RATIO) / SHOOTER_BULLETS_PER_DIAL)

/** 卡弹反转距离 = 半颗弹的编码器行程 */
#define SHOOTER_JAM_REVERSE_DISTANCE    (SHOOTER_ENCODER_PER_BULLET / 2.0f)

/* ========================================================================== */
/*                          卡弹检测参数                                        */
/* ========================================================================== */

/** 卡弹判定：转速低于此阈值（RPM）*/
#define SHOOTER_JAM_OMEGA_THRESHOLD     10U

/** 卡弹判定：转矩电流高于此阈值（mA，即堵转）*/
#define SHOOTER_JAM_TORQUE_THRESHOLD    6000U

/** 卡弹判定：连续满足条件的帧数阈值 */
#define SHOOTER_JAM_COUNT_THRESHOLD     10U

/* ========================================================================== */
/*                          位置 / 速度控制参数                                  */
/* ========================================================================== */

/** 认为"到位"的误差范围（编码器值），低于此值切换制动 */
#define SHOOTER_NEAR_THRESHOLD          360.0f

/** 认为"停稳"的转速阈值（RPM），到位后等待此条件进入制动 */
#define SHOOTER_STOP_OMEGA_THRESHOLD    200.0f

/** 单发完成后确认静止的转速阈值（RPM）*/
#define SHOOTER_IDLE_OMEGA_THRESHOLD    100.0f

/** 位置环输出（目标转速）限幅（RPM）*/
#define SHOOTER_OMEGA_CLAMP             3000.0f

/* ========================================================================== */
/*                          PID 参数（拨弹盘 M2006 串级）                       */
/* ========================================================================== */

/* ── 外环：位置环 ── */
#define SHOOTER_ANGLE_KP            1.0f
#define SHOOTER_ANGLE_KI            0.0f
#define SHOOTER_ANGLE_KD            0.0f
#define SHOOTER_ANGLE_IOUT_MAX      500.0f  /**< 位置环积分限幅（RPM）  */
#define SHOOTER_ANGLE_OUT_MAX       SHOOTER_OMEGA_CLAMP /**< 位置环输出限幅（RPM）*/
#define SHOOTER_ANGLE_IGAP          500.0f  /**< 位置环积分分离阈值     */

/* ── 内环：速度环 ── */
#define SHOOTER_OMEGA_KP            12.0f
#define SHOOTER_OMEGA_KI            0.014f
#define SHOOTER_OMEGA_KD            0.0f
#define SHOOTER_OMEGA_IOUT_MAX      1000.0f /**< 速度环积分限幅（电流） */
#define SHOOTER_OMEGA_OUT_MAX       5000.0f /**< 速度环输出限幅（电流） */
#define SHOOTER_OMEGA_IGAP          50.0f   /**< 速度环积分分离阈值     */

/* ========================================================================== */
/*                              数据结构定义                                    */
/* ========================================================================== */

/**
 * @brief  拨弹盘控制器内部状态
 * @note   由 shooter.c 内部持有，外部不直接操作此结构体。
 */
typedef struct
{
    float target_angle; /**< 当前目标累积编码器值（位置环参考量）*/
    PID_t pid_angle;    /**< 外环：位置 PID                       */
    PID_t pid_omega;    /**< 内环：速度 PID                       */
} Shooter_t;

/* ========================================================================== */
/*                          全局数据（外部只读）                                */
/* ========================================================================== */

/**
 * @brief  热量控制实例（对外暴露供裁判系统模块读取当前热量状态）
 * @note   仅供读取，禁止外部直接写入。
 */
extern HeatControl_t g_heat_ctrl;

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  发射机构初始化
 * @note   调用顺序要求：FrictionWheel_Init() → Shooter_Init()
 *         完成以下工作：
 *           1. 摩擦轮设为待机脉宽
 *           2. 热量控制初始化（使用默认赛规参数）
 *           3. 单发 / 卡弹状态机复位
 *           4. 拨弹盘串级 PID 初始化
 * @retval 无
 */
void Shooter_Init(void);

/**
 * @brief  查询发射机构是否处于启用状态
 * @retval 1: 已启用，0: 已禁用（安全模式）
 */
uint8_t Shooter_IsEnabled(void);

/**
 * @brief  设置发射机构启用/禁用状态
 * @note   启用时：摩擦轮切换为预转脉宽
 *         禁用时：摩擦轮切换为待机脉宽，单发和卡弹状态机强制复位
 * @param  enabled  1=启用，0=禁用
 * @retval 无
 */
void Shooter_SetEnabled(uint8_t enabled);

/**
 * @brief  发射机构主控制函数（每帧调用，须以 1kHz 固定频率调用）
 * @note   控制优先级（从高到低）：
 *           1. 未启用    → 等待安全键触发启用
 *           2. 卡弹恢复中 → 优先执行反转恢复
 *           3. 连发键按住 → 连发控制 + 卡弹检测
 *           4. 连发键松开 → 停转，清单发标志
 *           5. 单发键操作 → 触发一次单发（松开时生效）
 *           6. 安全键     → 关闭发射机构
 *           7. 空闲/单发执行中
 *         按键历史统一在函数末尾更新，各分支不重复赋值（do-while(0) 结构）。
 * @param  safety_btn  安全键：上升沿切换启用/禁用
 * @param  fire_btn    连发键：按住连发，松开停转
 * @param  once_btn    单发键：松开时触发一次
 * @retval 无
 */
void Shooter_Control(uint8_t safety_btn, uint8_t fire_btn, uint8_t once_btn);

#endif /* SHOOTER_H */
