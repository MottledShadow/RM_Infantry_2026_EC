/**
 ******************************************************************************
 * @file    vtm_driver.c
 * @brief   VTM 视觉模块设备驱动实现（设备驱动层）
 * @note    职责边界：
 *            - 覆盖 BSP_UART_VTM_DataReadyCallback() 接入原始数据
 *            - 解析帧字节到 g_vtm_ctrl
 *            - 维护离线检测与数据复位
 *          【禁止】在本文件中调用控制层或应用层接口
 * @version 1.0
 * @date    2026-4-4
 * @author  MOS
 ******************************************************************************
 */

#include "vtm_driver.h"

/* ========================================================================== */
/*                          全局控制数据与离线状态                              */
/* ========================================================================== */

VTM_ctrl_t g_vtm_ctrl;

static uint32_t s_vtm_last_rx_tick = 0U;
static uint8_t  s_vtm_is_offline   = 1U;

/* ========================================================================== */
/*                              内部函数                                        */
/* ========================================================================== */

/**
 * @brief  将所有控制字段复位为安全中立状态
 */
static void VTM_ResetData(void)
{
    for (uint8_t i = 0U; i < VTM_CHANNEL_COUNT; i++)
        g_vtm_ctrl.rc.channel[i] = VTM_CHANNEL_CENTER;

    g_vtm_ctrl.rc.mode_switch      = 0U;
    g_vtm_ctrl.rc.pause_button     = 0U;
    g_vtm_ctrl.rc.custom_button[0] = 0U;
    g_vtm_ctrl.rc.custom_button[1] = 0U;
    g_vtm_ctrl.rc.dial             = VTM_CHANNEL_CENTER;
    g_vtm_ctrl.rc.trigger          = 0U;

    g_vtm_ctrl.mouse.x_axis        = 0;
    g_vtm_ctrl.mouse.y_axis        = 0;
    g_vtm_ctrl.mouse.z_axis        = 0;
    g_vtm_ctrl.mouse.left_button   = 0U;
    g_vtm_ctrl.mouse.right_button  = 0U;
    g_vtm_ctrl.mouse.middle_button = 0U;

    g_vtm_ctrl.keyboard.w     = 0U; g_vtm_ctrl.keyboard.s = 0U;
    g_vtm_ctrl.keyboard.a     = 0U; g_vtm_ctrl.keyboard.d = 0U;
    g_vtm_ctrl.keyboard.shift = 0U; g_vtm_ctrl.keyboard.ctrl = 0U;
    g_vtm_ctrl.keyboard.q     = 0U; g_vtm_ctrl.keyboard.e = 0U;
    g_vtm_ctrl.keyboard.r     = 0U; g_vtm_ctrl.keyboard.f = 0U;
    g_vtm_ctrl.keyboard.g     = 0U; g_vtm_ctrl.keyboard.z = 0U;
    g_vtm_ctrl.keyboard.x     = 0U; g_vtm_ctrl.keyboard.c = 0U;
    g_vtm_ctrl.keyboard.v     = 0U; g_vtm_ctrl.keyboard.b = 0U;
}

/**
 * @brief  解析 VTM 帧原始字节到控制结构体
 * @param  buf  指向原始接收缓冲区的指针
 */
static void VTM_ParseFrame(const uint8_t *buf)
{
    /* ── 摇杆通道（11 位跨字节位域）── */
    g_vtm_ctrl.rc.channel[0] = (uint16_t)((buf[2] | ((uint16_t)buf[3] << 8))
                                           & VTM_CHANNEL_MASK);
    g_vtm_ctrl.rc.channel[1] = (uint16_t)(((uint16_t)buf[3] >> 3 | ((uint16_t)buf[4] << 5))
                                           & VTM_CHANNEL_MASK);
    g_vtm_ctrl.rc.channel[2] = (uint16_t)(((uint16_t)buf[4] >> 6 | ((uint16_t)buf[5] << 2)
                                           | ((uint16_t)buf[6] << 10))
                                           & VTM_CHANNEL_MASK);
    g_vtm_ctrl.rc.channel[3] = (uint16_t)(((uint16_t)buf[6] >> 1 | ((uint16_t)buf[7] << 7))
                                           & VTM_CHANNEL_MASK);

    /* ── 模式 / 按键（buf[7] 高位复合字段）── */
    g_vtm_ctrl.rc.mode_switch      = (buf[VTM_MODE_BYTE] >> 4) & 0x03U;
    g_vtm_ctrl.rc.pause_button     = (buf[VTM_MODE_BYTE] >> 6) & 0x01U;
    g_vtm_ctrl.rc.custom_button[0] =  buf[VTM_MODE_BYTE] >> 7;
    g_vtm_ctrl.rc.custom_button[1] =  buf[VTM_CUSTOM_BYTE] & 0x01U;

    /* ── 拨盘（11 位跨字节）── */
    g_vtm_ctrl.rc.dial    = (uint16_t)(((uint16_t)buf[VTM_CUSTOM_BYTE] >> 1
                            | ((uint16_t)buf[VTM_DIAL_H_BYTE] << 7))
                            & VTM_CHANNEL_MASK);
    g_vtm_ctrl.rc.trigger = (buf[VTM_DIAL_H_BYTE] >> 4) & 0x01U;

    /* ── 鼠标轴（16 位有符号，小端）── */
    g_vtm_ctrl.mouse.x_axis = (int16_t)(buf[VTM_MOUSE_X_L_BYTE]
                             | ((uint16_t)buf[VTM_MOUSE_X_L_BYTE + 1U] << 8));
    g_vtm_ctrl.mouse.y_axis = (int16_t)(buf[12]
                             | ((uint16_t)buf[13] << 8));
    g_vtm_ctrl.mouse.z_axis = (int16_t)(buf[14]
                             | ((uint16_t)buf[15] << 8));

    /* ── 鼠标按键（buf[16] 各 2 位）── */
    g_vtm_ctrl.mouse.left_button   =  buf[VTM_MOUSE_BTN_BYTE]       & 0x03U;
    g_vtm_ctrl.mouse.right_button  = (buf[VTM_MOUSE_BTN_BYTE] >> 2) & 0x03U;
    g_vtm_ctrl.mouse.middle_button = (buf[VTM_MOUSE_BTN_BYTE] >> 4) & 0x03U;

    /* ── 键盘（buf[17] 逐位，buf[18] 逐位）── */
    g_vtm_ctrl.keyboard.w     =  buf[VTM_KB_BYTE1]       & 0x01U;
    g_vtm_ctrl.keyboard.s     = (buf[VTM_KB_BYTE1] >> 1) & 0x01U;
    g_vtm_ctrl.keyboard.a     = (buf[VTM_KB_BYTE1] >> 2) & 0x01U;
    g_vtm_ctrl.keyboard.d     = (buf[VTM_KB_BYTE1] >> 3) & 0x01U;
    g_vtm_ctrl.keyboard.shift = (buf[VTM_KB_BYTE1] >> 4) & 0x01U;
    g_vtm_ctrl.keyboard.ctrl  = (buf[VTM_KB_BYTE1] >> 5) & 0x01U;
    g_vtm_ctrl.keyboard.q     = (buf[VTM_KB_BYTE1] >> 6) & 0x01U;
    g_vtm_ctrl.keyboard.e     = (buf[VTM_KB_BYTE1] >> 7) & 0x01U;

    g_vtm_ctrl.keyboard.r     =  buf[VTM_KB_BYTE2]       & 0x01U;
    g_vtm_ctrl.keyboard.f     = (buf[VTM_KB_BYTE2] >> 1) & 0x01U;
    g_vtm_ctrl.keyboard.g     = (buf[VTM_KB_BYTE2] >> 2) & 0x01U;
    g_vtm_ctrl.keyboard.z     = (buf[VTM_KB_BYTE2] >> 3) & 0x01U;
    g_vtm_ctrl.keyboard.x     = (buf[VTM_KB_BYTE2] >> 4) & 0x01U;
    g_vtm_ctrl.keyboard.c     = (buf[VTM_KB_BYTE2] >> 5) & 0x01U;
    g_vtm_ctrl.keyboard.v     = (buf[VTM_KB_BYTE2] >> 6) & 0x01U;
    g_vtm_ctrl.keyboard.b     = (buf[VTM_KB_BYTE2] >> 7) & 0x01U;
}

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  检查 VTM 视觉模块是否离线
 */
uint8_t VTM_CheckOffline(uint32_t timeout_ms)
{
    if (s_vtm_is_offline)
        return 1U;

    if ((HAL_GetTick() - s_vtm_last_rx_tick) > timeout_ms)
    {
        s_vtm_is_offline = 1U;
        VTM_ResetData();
    }

    return s_vtm_is_offline;
}

/* ========================================================================== */
/*                    覆盖 BSP_UART __weak 回调                                 */
/* ========================================================================== */

/**
 * @brief  覆盖 bsp_uart.c 中的弱符号回调，接入 VTM 帧解析
 */
void BSP_UART_VTM_DataReadyCallback(uint8_t *buf, uint16_t size)
{
    (void)size;
    VTM_ParseFrame(buf);
    s_vtm_last_rx_tick = HAL_GetTick();
    s_vtm_is_offline   = 0U;
}
