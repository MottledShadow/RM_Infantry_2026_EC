/**
 ******************************************************************************
 * @file    motor_driver.h
 * @brief   电机设备驱动头文件（设备驱动层）
 * @note    层级说明：
 *            本文件属于【设备驱动层】，位于 BSP 层之上、控制层之下。
 *            职责：覆盖 BSP_CAN_RxFrameCallback()，将 CAN 原始字节
 *                  解析为电机工程量数据结构，供控制层只读访问。
 *          数据流向：
 *            HAL CAN 中断 → bsp_can（取帧，触发 weak 回调）
 *                         → motor_driver（覆盖回调，解析帧）
 *                         → 控制层（读取 g_motor_xxx）
 * @version 1.1
 * @date    2026-4-4
 * @author  MOS
 ******************************************************************************
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "main.h"
#include <stdint.h>
#include "bsp_can.h"  /* 仅用于 CAN ID 宏定义，依赖方向（驱动层 → BSP 层）*/

/* ========================================================================== */
/*                              电机数量宏定义                                  */
/* ========================================================================== */

#define M3508_MOTOR_COUNT   4U  /**< 底盘 3508 电机数量 */
#define M6020_MOTOR_COUNT   2U  /**< 云台 6020 电机数量 */

/* ========================================================================== */
/*                          CAN 数据字段偏移量                                  */
/* ========================================================================== */

#define MOTOR_ANGLE_H_BYTE      0U  /**< 机械角度高字节偏移 */
#define MOTOR_ANGLE_L_BYTE      1U  /**< 机械角度低字节偏移 */
#define MOTOR_SPEED_H_BYTE      2U  /**< 转速高字节偏移     */
#define MOTOR_SPEED_L_BYTE      3U  /**< 转速低字节偏移     */
#define MOTOR_CURRENT_H_BYTE    4U  /**< 转矩电流高字节偏移 */
#define MOTOR_CURRENT_L_BYTE    5U  /**< 转矩电流低字节偏移 */
#define MOTOR_TEMP_BYTE         6U  /**< 温度字节偏移       */
#define MOTOR_ERR_BYTE          7U  /**< 错误码字节偏移     */

/* ========================================================================== */
/*                              解析标志位                                      */
/* ========================================================================== */

#define MOTOR_PARSE_FLAG_TEMP   0x01U  /**< 包含温度字段（3508, 6020）   */
#define MOTOR_PARSE_FLAG_ERR    0x02U  /**< 包含错误码字段（3508, 2006） */

#define MOTOR_3508_PARSE_FLAGS  (MOTOR_PARSE_FLAG_TEMP | MOTOR_PARSE_FLAG_ERR)
#define MOTOR_2006_PARSE_FLAGS  (MOTOR_PARSE_FLAG_ERR)
#define MOTOR_6020_PARSE_FLAGS  (MOTOR_PARSE_FLAG_TEMP)

/* ========================================================================== */
/*                              数据结构定义                                    */
/* ========================================================================== */

/**
 * @brief  单个电机反馈数据结构
 */
typedef struct
{
    uint16_t rotor_mechanical_angle;     /**< 转子机械角度，范围 [0, 8191]，对应 [0°, 360°) */
    int16_t  rotor_speed;                /**< 转子转速，单位 rpm                              */
    int16_t  actual_quadrature_current;  /**< 实际转矩电流，单位 mA                           */
    uint8_t  motor_temperature;          /**< 电机温度，单位 ℃（部分型号支持）                */
    uint8_t  error_codes;                /**< 错误码（部分型号支持）                           */
} Motor_Measure_t;

/* ========================================================================== */
/*                          全局电机状态（控制层只读）                           */
/* ========================================================================== */

extern volatile Motor_Measure_t g_motor_3508[M3508_MOTOR_COUNT]; /**< 底盘 3508 x4 */
extern volatile Motor_Measure_t g_motor_6020[M6020_MOTOR_COUNT]; /**< 云台 6020 x2 */
extern volatile Motor_Measure_t g_motor_2006;                     /**< 拨弹盘 2006  */

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  CAN 帧分发与电机数据解析（内部入口，可供单元测试调用）
 * @param  hcan   接收到该帧的 CAN 总线句柄
 * @param  stdId  接收帧CAN ID
 * @param  data   8 字节原始数据指针
 */
void Motor_ParseCANFrame(CAN_HandleTypeDef *hcan, uint32_t stdId, const uint8_t *data);

#endif /* MOTOR_DRIVER_H */
