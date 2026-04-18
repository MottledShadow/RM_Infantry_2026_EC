/**
 ******************************************************************************
 * @file    filter.h
 * @brief   CAN 总线过滤器初始化头文件（BSP 层）
 * @note    层级说明：
 *            本文件属于【BSP 层】，直接配置 STM32 CAN 硬件过滤器。
 *            职责：启动 CAN1 / CAN2 总线并配置接收过滤规则，
 *                  使所有标准帧均可通过（掩码全 0）进入 FIFO0，
 *                  触发 CAN_IT_RX_FIFO0_MSG_PENDING 中断。
 *
 *          调用时机：
 *            在 BSP_Init() 中调用，或在 MX_CAN_Init() 之后、
 *            进入主循环之前调用。
 *
 *          TODO: 如需按帧 ID 过滤（白名单），修改 FilterIdHigh/Low
 *                与 FilterMaskIdHigh/Low 的配置值。
 *
 * @version 1.0
 * @date    2022-9-3
 * @author  DJI
 ******************************************************************************
 */

#ifndef __FILTER_H__
#define __FILTER_H__

#include "bsp.h"

void can_filter_init(void);

#endif
