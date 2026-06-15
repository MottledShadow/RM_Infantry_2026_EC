/**
 ******************************************************************************
 * @file    bsp_debug.c
 * @brief   调试工具实现（BSP 层）
 * @note    SerialPlot 帧格式（小端字节序）：
 *            Byte 0       : 0xAB（帧头）
 *            Byte 1~4     : float[0] 的 4 字节原始内存（小端）
 *            Byte 5~8     : float[1] ...
 *            ...
 *          上位机工具：SerialPlot（https://github.com/hyOzd/serialplot）
 * @version 1.0
 * @date    2026-4-4
 * @author  MOS
 ******************************************************************************
 */

#include "bsp_debug.h"

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  通过 UART 向 SerialPlot 上位机发送浮点数据帧
 */
void BSP_Debug_SerialPlot(UART_HandleTypeDef *huart, float *data, uint16_t numChannels)
{
    /* 发送缓冲区：静态分配，避免栈溢出 */
    static uint8_t txBuf[SERIAL_PLOT_BUF_SIZE];

    /* 限制通道数，防止越界 */
    if (numChannels > SERIAL_PLOT_MAX_CHANNELS)
    {
        numChannels = SERIAL_PLOT_MAX_CHANNELS;
    }

    /* 填写帧头 */
    txBuf[0] = SERIAL_PLOT_FRAME_HEADER;

    /* 将每个 float 按原始内存字节序（小端）写入缓冲区 */
    for (uint16_t i = 0; i < numChannels; i++)
    {
        /* 取 float 的原始字节指针，逐字节拷贝 */
        const uint8_t *pFloat = (const uint8_t *)(&data[i]);
        uint16_t offset = (uint16_t)(i * SERIAL_PLOT_FLOAT_BYTES + 1U);

        txBuf[offset + 0U] = pFloat[0];
        txBuf[offset + 1U] = pFloat[1];
        txBuf[offset + 2U] = pFloat[2];
        txBuf[offset + 3U] = pFloat[3];
    }

    /* 通过 UART DMA 发送 */
    BSP_UART_TxData(huart, txBuf, (uint16_t)(numChannels * SERIAL_PLOT_FLOAT_BYTES + 1U));
}
