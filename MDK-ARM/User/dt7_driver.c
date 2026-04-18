/**
 ******************************************************************************
 * @file    dt7_driver.c
 * @brief   DT7 遥控器设备驱动实现（设备驱动层）
 * @note    职责边界：
 *            - 覆盖 BSP_UART_DT7_DataReadyCallback() 接入 SBUS 数据
 *            - 解析 SBUS 原始字节到 g_dt7_ctrl
 *            - 维护离线检测与数据复位
 *          【禁止】在本文件中调用控制层或应用层接口
 * @version 1.0
 * @date    2026-4-4
 * @author  MOS
 ******************************************************************************
 */

#include "dt7_driver.h"

/* ========================================================================== */
/*                          全局控制数据与离线状态                              */
/* ========================================================================== */

DT7_ctrl_t g_dt7_ctrl;

static uint32_t s_dt7_last_rx_tick = 0U; /**< 最后一次成功接收的系统 Tick */
static uint8_t  s_dt7_is_offline   = 1U; /**< 离线标志，初始为离线        */

/* ========================================================================== */
/*                              内部函数                                        */
/* ========================================================================== */

/**
 * @brief  将所有通道值复位为安全中立状态
 * @note   在离线判定时调用，防止控制层读取到残余的非零拨杆数据
 */
static void DT7_ResetData(void)
{
    for (uint8_t i = 0U; i < DT7_CHANNEL_COUNT; i++)
    {
        g_dt7_ctrl.rc.ch[i] = (int16_t)DT7_CHANNEL_CENTER;
    }
    g_dt7_ctrl.rc.s[0] = 0;
    g_dt7_ctrl.rc.s[1] = 0;
}

/**
 * @brief  从 SBUS 字节流中提取 11 位通道值并填充控制结构体
 * @param  buf  指向 SBUS 原始缓冲区的指针
 */
static void DT7_ParseSBUS(const uint8_t *buf)
{
    /* SBUS 11 位通道按位域跨字节存储，需逐字节组合后掩码截取 */
    g_dt7_ctrl.rc.ch[0] =  (int16_t)((buf[0]
                          | ((uint16_t)buf[1] << 8))
                          & DT7_SBUS_CHANNEL_MASK);

    g_dt7_ctrl.rc.ch[1] =  (int16_t)(((uint16_t)buf[1] >> 3
                          | ((uint16_t)buf[2] << 5))
                          & DT7_SBUS_CHANNEL_MASK);

    g_dt7_ctrl.rc.ch[2] =  (int16_t)(((uint16_t)buf[2] >> 6
                          | ((uint16_t)buf[3] << 2)
                          | ((uint16_t)buf[4] << 10))
                          & DT7_SBUS_CHANNEL_MASK);

    g_dt7_ctrl.rc.ch[3] =  (int16_t)(((uint16_t)buf[4] >> 1
                          | ((uint16_t)buf[5] << 7))
                          & DT7_SBUS_CHANNEL_MASK);

    g_dt7_ctrl.rc.ch[4] =  (int16_t)((buf[16]
                          | ((uint16_t)buf[17] << 8))
                          & DT7_SBUS_CHANNEL_MASK);

    /* 拨杆：2 位拨档值从 buf[5] 高 4 位中分别提取 */
    g_dt7_ctrl.rc.s[0] = (char)(((buf[5] >> 4) & 0x0CU) >> 2U);
    g_dt7_ctrl.rc.s[1] = (char)( (buf[5] >> 4) & 0x03U);
}

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  检查 DT7 遥控器是否离线
 */
uint8_t DT7_CheckOffline(uint32_t timeout_ms)
{
    if (s_dt7_is_offline)
        return 1U;

    if ((HAL_GetTick() - s_dt7_last_rx_tick) > timeout_ms)
    {
        s_dt7_is_offline = 1U;
        DT7_ResetData();  /* 立即清零，防止残余数据流入控制层 */
    }

    return s_dt7_is_offline;
}

/* ========================================================================== */
/*                    覆盖 BSP_UART __weak 回调                                 */
/* ========================================================================== */

/**
 * @brief  覆盖 bsp_uart.c 中的弱符号回调，接入 DT7 SBUS 解析
 * @note   由 HAL_UARTEx_RxEventCallback 在数据就绪后调用
 */
void BSP_UART_DT7_DataReadyCallback(uint8_t *buf, uint16_t size)
{
    (void)size; /* SBUS 帧长固定，不依赖 size 参数 */
    DT7_ParseSBUS(buf);
    s_dt7_last_rx_tick = HAL_GetTick();
    s_dt7_is_offline   = 0U;
}
