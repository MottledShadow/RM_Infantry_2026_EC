/**
 ******************************************************************************
 * @file    friction_wheel.c
 * @brief   摩擦轮控制实现（控制层）
 * @note    职责边界：
 *            - 将业务状态（Idle / Ready / Fire）映射为 PWM 脉宽
 *            - 通过 bsp_pwm 驱动两路摩擦轮电调
 *          【禁止】在本文件中直接调用 HAL_TIM_* / __HAL_TIM_*
 *                  所有 PWM 操作均通过 bsp_pwm.h 接口完成
 * @version 1.0
 * @date    2026-4-5
 * @author  黄仲华
 ******************************************************************************
 */

#include "friction_wheel.h"

/* ========================================================================== */
/*                              内部辅助函数                                    */
/* ========================================================================== */

/**
 * @brief  同步设置左右两路摩擦轮脉宽（内部复用）
 */
static inline void FW_SetBothPulse(uint16_t pulse)
{
    BSP_PWM_SetPulse(&BSP_PWM_FW_LEFT,  pulse);
    BSP_PWM_SetPulse(&BSP_PWM_FW_RIGHT, pulse);
}

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  摩擦轮控制层初始化
 */
void FrictionWheel_Init(void)
{
    /* 启动两路 PWM 通道 */
    BSP_PWM_Start(&BSP_PWM_FW_LEFT);
    BSP_PWM_Start(&BSP_PWM_FW_RIGHT);

    /* 使能高级定时器主输出（TIM8 为高级定时器，必须使能 MOE）*/
    BSP_PWM_EnableMOE(&htim8);

    /* 输出待机脉宽，完成电调初始化握手 */
    FW_SetBothPulse(FRICTION_IDLE_PULSE);
}

/**
 * @brief  设置摩擦轮双路 PWM 脉宽（同步）
 */
void FrictionWheel_SetPulse(uint16_t pulse)
{
    FW_SetBothPulse(pulse);
}

/**
 * @brief  设置摩擦轮为预转状态
 */
void FrictionWheel_SetReady(void)
{
    FW_SetBothPulse(FRICTION_READY_PULSE);
}

/**
 * @brief  设置摩擦轮为发射转速
 */
void FrictionWheel_SetFire(void)
{
    FW_SetBothPulse(FRICTION_FIRE_PULSE);
}

/**
 * @brief  设置摩擦轮为待机（停转）
 */
void FrictionWheel_SetIdle(void)
{
    FW_SetBothPulse(FRICTION_IDLE_PULSE);
}
