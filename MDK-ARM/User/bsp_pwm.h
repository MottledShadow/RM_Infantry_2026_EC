/**
 ******************************************************************************
 * @file    bsp_pwm.h
 * @brief   PWM 底层驱动头文件（BSP 层）
 * @note    层级说明：
 *            本文件属于【BSP 层】，封装 HAL 定时器 PWM 原语。
 *            上层（控制层）通过通道句柄结构体操作 PWM，
 *            不直接接触 htim / TIM_CHANNEL_x 等 HAL 细节。
 *
 *          当前硬件映射（TODO: 如有变化请在此处同步修改）：
 *            摩擦轮 PWM → TIM8 CH1 + CH2
 *
 * @version 1.0
 * @date    2026-4-5
 * @author  黄仲华
 ******************************************************************************
 */

#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "main.h"
#include "tim.h"
#include <stdint.h>

/* ========================================================================== */
/*                          PWM 通道句柄                                        */
/* ========================================================================== */

/**
 * @brief  PWM 通道描述结构体
 * @note   上层持有此结构体实例，调用 BSP_PWM_* 系列函数传入即可，
 *         无需关心 htim 指针或 TIM_CHANNEL_x 宏。
 */
typedef struct
{
    TIM_HandleTypeDef *htim;     /**< 所属定时器句柄  */
    uint32_t           channel;  /**< HAL 通道宏，如 TIM_CHANNEL_1 */
} BSP_PWM_Channel_t;

/* ========================================================================== */
/*                          预定义通道（外部只读）                               */
/* ========================================================================== */

/**
 * @brief  摩擦轮 PWM 通道（左轮 TIM8 CH1，右轮 TIM8 CH2）
 * @note   由 bsp_pwm.c 定义，上层通过指针访问，不直接操作内部字段。
 */
extern const BSP_PWM_Channel_t BSP_PWM_FW_LEFT;   /**< 摩擦轮左轮 PWM 通道 */
extern const BSP_PWM_Channel_t BSP_PWM_FW_RIGHT;  /**< 摩擦轮右轮 PWM 通道 */

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  启动单个 PWM 通道输出
 * @param  ch  目标 PWM 通道句柄指针
 * @retval 无
 */
void BSP_PWM_Start(const BSP_PWM_Channel_t *ch);

/**
 * @brief  设置单个 PWM 通道的比较值（脉宽）
 * @param  ch     目标 PWM 通道句柄指针
 * @param  pulse  比较寄存器值（脉宽），单位取决于定时器配置
 * @retval 无
 */
void BSP_PWM_SetPulse(const BSP_PWM_Channel_t *ch, uint16_t pulse);

/**
 * @brief  启动指定定时器的互补 PWM 主输出（MOE）
 * @note   用于带死区控制的高级定时器（TIM1 / TIM8）
 *         普通定时器无需调用此函数。
 * @param  htim  目标定时器句柄指针
 * @retval 无
 */
void BSP_PWM_EnableMOE(TIM_HandleTypeDef *htim);

#endif /* BSP_PWM_H */
