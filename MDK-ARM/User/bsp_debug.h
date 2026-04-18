/**
 ******************************************************************************
 * @file    bsp_debug.h
 * @brief   调试工具头文件（BSP 层）
 * @note    提供 SerialPlot 上位机可视化调试接口。
 *          仅用于开发调试阶段，正式发布时可通过宏关闭。
 * @version 1.0
 * @date    2026-4-4
 * @author  MOS
 ******************************************************************************
 */

#ifndef BSP_DEBUG_H
#define BSP_DEBUG_H

#include "main.h"
#include <stdint.h>
#include "bsp_uart.h"

/* ========================================================================== */
/*                          SerialPlot 协议参数                                 */
/* ========================================================================== */

#define SERIAL_PLOT_FRAME_HEADER    0xABU   /**< 帧头标识字节 */
#define SERIAL_PLOT_MAX_CHANNELS    12U     /**< 最多支持同时显示的通道数 */
#define SERIAL_PLOT_FLOAT_BYTES     4U      /**< 每个 float 占用字节数（= sizeof(float)）*/

/** 发送缓冲区长度 = 1字节帧头 + N通道 × 4字节/float */
#define SERIAL_PLOT_BUF_SIZE        (SERIAL_PLOT_MAX_CHANNELS * SERIAL_PLOT_FLOAT_BYTES + 1U)

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  通过 UART 向 SerialPlot 上位机发送浮点数据帧
 * @note   数据帧格式：[0xAB][float0低字节...高字节][float1...]...
 *         numChannels 不得超过 SERIAL_PLOT_MAX_CHANNELS，
 *         否则仅发送前 SERIAL_PLOT_MAX_CHANNELS 个通道。
 * @param  huart        目标 UART 句柄指针
 * @param  data         指向浮点数组的指针
 * @param  numChannels  通道数（数组元素个数）
 * @retval 无
 */
void BSP_Debug_SerialPlot(UART_HandleTypeDef *huart, float *data, uint16_t numChannels);

#endif /* BSP_DEBUG_H */
