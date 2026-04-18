/**
 ******************************************************************************
 * @file    bsp_can.h
 * @brief   CAN 总线底层驱动头文件（BSP 层）
 * @note    本层仅负责 CAN 帧的收发，不做任何业务/协议解析。
 *          通过 __weak 回调向上层（设备驱动层）分发原始帧，
 *          BSP 层对驱动层实现完全透明，不产生向上依赖。
 *
 *          【架构说明】
 *          依赖方向：motor_driver（驱动层）→ bsp_can（BSP 层）
 *          BSP 层通过 __weak BSP_CAN_RxFrameCallback() 通知上层，
 *          与 bsp_uart 的 __weak 回调模式完全对称一致。
 *
 * @version 1.1
 * @date    2026-4-4
 * @author  MOS
 ******************************************************************************
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"
#include <stdint.h>
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* ========================================================================== */
/*                              CAN ID 宏定义                                  */
/* ========================================================================== */

/* ── CAN 总线句柄 ── */
#define GIMBAL_CAN_HANDLE   (&hcan2)    /**< 云台控制使用的 CAN 总线句柄 */
#define CHASSIS_CAN_HANDLE  (&hcan1)    /**< 底盘控制使用的 CAN 总线句柄 */
#define SHOOTER_CAN_HANDLE  (&hcan2)    /**< 拨弹盘控制使用的 CAN 总线句柄 */

/* ── 电机控制帧 ID ── */
#define GM6020_CTRL_ID_GROUP1       0x1FEU  /**< GM6020 电机ID 1~4 控制帧 ID */
#define GM6020_CTRL_ID_GROUP2       0x2FEU  /**< GM6020 电机ID 5~7 控制帧 ID */
#define M2006_M3508_CTRL_ID_GROUP1  0x200U   /**< M2006 和 M3508 电机ID 1~4 控制帧 ID */
#define M2006_M3508_CTRL_ID_GROUP2  0x1FFU   /**< M2006 和 M3508 电机ID 5~8 控制帧 ID */

/* ── CAN1 挂载电机 ID ── */
#define CAN1_M3508_1_ID     0x201U  /**< CAN1: 3508 电机 1 反馈帧 ID */
#define CAN1_M3508_2_ID     0x202U  /**< CAN1: 3508 电机 2 反馈帧 ID */
#define CAN1_M3508_3_ID     0x203U  /**< CAN1: 3508 电机 3 反馈帧 ID */
#define CAN1_M3508_4_ID     0x204U  /**< CAN1: 3508 电机 4 反馈帧 ID */

/* ── CAN2 挂载电机 ID ── */
#define CAN2_M2006_ID       0x201U  /**< CAN2: 2006 电机（拨弹盘）反馈帧 ID */
#define CAN2_M6020_1_ID     0x205U  /**< CAN2: 6020 电机 1（PIT）反馈帧 ID */
#define CAN2_M6020_2_ID     0x206U  /**< CAN2: 6020 电机 2（YAW）反馈帧 ID */
#define CAN2_M6020_3_ID     0x207U  /**< CAN2: 6020 电机 3（预留）反馈帧 ID */
#define CAN2_M6020_4_ID     0x208U  /**< CAN2: 6020 电机 4（预留）反馈帧 ID */

/* ── 帧格式 ── */
#define CAN_STD_DLC         0x08U   /**< 标准数据帧数据长度（8 字节） */

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  通过指定 CAN 总线发送一帧标准数据帧
 * @param  hcan   目标 CAN 总线句柄指针（&hcan1 或 &hcan2）
 * @param  stdId  发送帧CAN ID
 * @param  data   指向待发送数据的指针，长度必须 = 8 字节
 * @retval 无
 */
void BSP_CAN_TxData(CAN_HandleTypeDef *hcan, uint32_t stdId, uint8_t *data);

/* ========================================================================== */
/*                    上层驱动接收回调（弱函数接口）                             */
/* ========================================================================== */

/**
 * @brief  CAN 原始帧接收回调（__weak，由设备驱动层覆盖实现）
 * @note   BSP 层在 HAL 中断中收到完整帧后调用此函数。
 *         上层 motor_driver.c 覆盖本函数，完成帧的分发与解析。
 *         BSP 层无需 include 任何驱动层头文件，层间完全解耦。
 *
 *  使用示例（在 motor_driver.c 中覆盖）：
 *    void BSP_CAN_RxFrameCallback(CAN_HandleTypeDef *hcan,
 *                                 uint32_t stdId, const uint8_t *data)
 *    {
 *        Motor_ParseCANFrame(hcan, stdId, data);
 *    }
 *
 * @param  hcan   接收到该帧的 CAN 总线句柄
 * @param  stdId  接收帧CAN ID
 * @param  data   指向 8 字节原始数据的指针
 */
void BSP_CAN_RxFrameCallback(CAN_HandleTypeDef *hcan,
                              uint32_t           stdId,
                              const uint8_t     *data);

#endif /* BSP_CAN_H */
