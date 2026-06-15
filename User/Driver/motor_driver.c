/**
 ******************************************************************************
 * @file    motor_driver.c
 * @brief   电机设备驱动实现（设备驱动层）
 * @note    【架构修正说明 v1.1】
 *            覆盖 bsp_can.h 中声明的 __weak BSP_CAN_RxFrameCallback()，
 *            实现驱动层对 BSP 层的单向依赖（驱动层 → BSP 层）。
 *            bsp_can.c 无需 include 本文件，层间彻底解耦。
 * @version 1.1
 * @date    2026-4-4
 * @author  MOS
 ******************************************************************************
 */

#include "motor_driver.h"

/* ========================================================================== */
/*                          全局电机反馈状态                                    */
/* ========================================================================== */

volatile Motor_Measure_t g_motor_3508[M3508_MOTOR_COUNT];
volatile Motor_Measure_t g_motor_6020[M6020_MOTOR_COUNT];
volatile Motor_Measure_t g_motor_2006;

/* ========================================================================== */
/*                              内部函数                                        */
/* ========================================================================== */

/**
 * @brief  将 8 字节原始 CAN 数据解析到电机结构体
 * @param  motor       目标电机数据结构指针
 * @param  data        原始 CAN 数据（8 字节）
 * @param  parseFlags  解析功能标志，参见 MOTOR_PARSE_FLAG_xxx
 */
static void Motor_ParseFeedback(volatile Motor_Measure_t *motor,
                                const uint8_t            *data,
                                uint8_t                   parseFlags)
{
    motor->rotor_mechanical_angle    = ((uint16_t)data[MOTOR_ANGLE_H_BYTE]   << 8)
                                     |  (uint16_t)data[MOTOR_ANGLE_L_BYTE];
    motor->rotor_speed               = ((int16_t) data[MOTOR_SPEED_H_BYTE]   << 8)
                                     |  (int16_t) data[MOTOR_SPEED_L_BYTE];
    motor->actual_quadrature_current = ((int16_t) data[MOTOR_CURRENT_H_BYTE] << 8)
                                     |  (int16_t) data[MOTOR_CURRENT_L_BYTE];

    if (parseFlags & MOTOR_PARSE_FLAG_TEMP)
        motor->motor_temperature = data[MOTOR_TEMP_BYTE];

    if (parseFlags & MOTOR_PARSE_FLAG_ERR)
        motor->error_codes = data[MOTOR_ERR_BYTE];
}

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  CAN 帧分发与电机数据解析
 */
void Motor_ParseCANFrame(CAN_HandleTypeDef *hcan, uint32_t stdId, const uint8_t *data)
{
    if (hcan == &hcan1)
    {
        switch (stdId)
        {
            case CAN1_M3508_1_ID: Motor_ParseFeedback(&g_motor_3508[0], data, MOTOR_3508_PARSE_FLAGS); break;
            case CAN1_M3508_2_ID: Motor_ParseFeedback(&g_motor_3508[1], data, MOTOR_3508_PARSE_FLAGS); break;
            case CAN1_M3508_3_ID: Motor_ParseFeedback(&g_motor_3508[2], data, MOTOR_3508_PARSE_FLAGS); break;
            case CAN1_M3508_4_ID: Motor_ParseFeedback(&g_motor_3508[3], data, MOTOR_3508_PARSE_FLAGS); break;
            default: break;
        }
    }
    else if (hcan == &hcan2)
    {
        switch (stdId)
        {
            case CAN2_M2006_ID:   Motor_ParseFeedback(&g_motor_2006,    data, MOTOR_2006_PARSE_FLAGS); break;
            case CAN2_M6020_1_ID: Motor_ParseFeedback(&g_motor_6020[0], data, MOTOR_6020_PARSE_FLAGS); break;
            case CAN2_M6020_2_ID: Motor_ParseFeedback(&g_motor_6020[1], data, MOTOR_6020_PARSE_FLAGS); break;
            case CAN2_M6020_3_ID: /* 预留 */ break;
            case CAN2_M6020_4_ID: /* 预留 */ break;
            default: break;
        }
    }
}

/* ========================================================================== */
/*                    覆盖 BSP_CAN __weak 回调                                  */
/* ========================================================================== */

/**
 * @brief  覆盖 bsp_can.c 中的弱符号回调，接入电机帧解析
 * @note   此处是驱动层与 BSP 层的唯一接合点。
 *         依赖方向：motor_driver（驱动层）→ bsp_can（BSP 层）。
 */
void BSP_CAN_RxFrameCallback(CAN_HandleTypeDef *hcan,
                              uint32_t           stdId,
                              const uint8_t     *data)
{
    Motor_ParseCANFrame(hcan, stdId, data);
}
