/**
 ******************************************************************************
 * @file    friction_wheel.h
 * @brief   摩擦轮控制头文件（控制层）
 * @note    层级说明：
 *            本文件属于【控制层】，位于 BSP 层之上、应用层之下。
 *            职责：将"待机 / 预转 / 发射"等业务状态映射为 PWM 脉宽值，
 *                  通过 bsp_pwm 驱动摩擦轮电调（ESC）。
 *            不包含任何 HAL 调用，硬件操作均委托给 bsp_pwm 层完成。
 *
 *          硬件说明：
 *            摩擦轮电调为 C615，
 *            脉宽范围约 1000~2000 μs，中点 1500 μs。
 *            具体脉宽与转速对应关系请根据实际标定结果修改下方宏。
 *
 * @version 1.0
 * @date    2026-4-5
 * @author  黄仲华
 ******************************************************************************
 */

#ifndef FRICTION_WHEEL_H
#define FRICTION_WHEEL_H

#include <stdint.h>
#include "bsp.h"  /* 控制层 → BSP 层，依赖方向正确 */

/* ========================================================================== */
/*                          PWM 脉宽状态定义                                    */
/* ========================================================================== */

/**
 * @brief  摩擦轮 PWM 脉宽枚举
 * @note   单位为定时器比较寄存器值，对应 PWM 占空比。
 *         实际物理脉宽 = pulse / TIM_ARR × 周期（μs）。
 *         TODO: 请根据实际标定结果调整以下数值。
 */
#define FRICTION_IDLE_PULSE     1000U  /**< 待机脉宽：电调初始化/停止转动 */
#define FRICTION_READY_PULSE    1200U  /**< 预转脉宽：低速预转，等待发射指令 */
#define FRICTION_FIRE_PULSE     1370U  /**< 发射脉宽：达到目标弹速的转速 */

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  摩擦轮控制层初始化
 * @note   启动左右两路 PWM 通道，使能 MOE，并输出待机脉宽。
 *         【调用时机】在应用层 App_Init() 中调用，不应在 BSP_Init() 中调用。
 * @retval 无
 */
void FrictionWheel_Init(void);

/**
 * @brief  设置摩擦轮双路 PWM 脉宽（同步）
 * @note   左右轮同步设置，保证弹道对称。
 *         如需差速控制请直接调用 BSP_PWM_SetPulse() 单独设置。
 * @param  pulse  脉宽值，建议使用 FRICTION_*_PULSE 宏或标定值
 * @retval 无
 */
void FrictionWheel_SetPulse(uint16_t pulse);

/**
 * @brief  设置摩擦轮为预转状态（等待发射）
 * @retval 无
 */
void FrictionWheel_SetReady(void);

/**
 * @brief  设置摩擦轮为发射转速
 * @retval 无
 */
void FrictionWheel_SetFire(void);

/**
 * @brief  设置摩擦轮为待机（停转）
 * @retval 无
 */
void FrictionWheel_SetIdle(void);

#endif /* FRICTION_WHEEL_H */
