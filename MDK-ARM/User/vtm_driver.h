/**
 ******************************************************************************
 * @file    vtm_driver.h
 * @brief   VTM 图传模块设备驱动头文件（设备驱动层）
 * @note    层级说明：
 *            本文件属于【设备驱动层】，位于 BSP 层之上、控制层之下。
 *            职责：覆盖 BSP_UART_VTM_DataReadyCallback()，
 *                  将图传模块 UART 原始字节解析为控制结构体，
 *                  并维护在线/离线状态检测。
 *          VTM 帧结构说明(详情请查看裁判系统相机图传模块使用说明书)：
 *            Byte 0~1   : 帧头（0xA9 0x53）
 *            Byte 2~9   : 通道 / 模式 / 按键位域
 *            Byte 10~16 : 鼠标轴值与按键
 *            Byte 17~18 : 键盘状态
 * @version 1.0
 * @date    2026-4-4
 * @author  MOS
 ******************************************************************************
 */

#ifndef VTM_DRIVER_H
#define VTM_DRIVER_H

#include "main.h"
#include <stdint.h>
#include "bsp_uart.h"  /* 覆盖 __weak 回调，依赖方向正确（驱动层 → BSP 层）*/

/* ========================================================================== */
/*                              协议常量定义                                    */
/* ========================================================================== */

/* ── 通道解析 ── */
#define VTM_CHANNEL_MASK        0x07FFU /**< 11 位通道值掩码                  */
#define VTM_CHANNEL_COUNT       4U      /**< 摇杆通道数量                     */
#define VTM_CHANNEL_CENTER      1024U   /**< 通道中立值（离线复位时使用）     */
#define VTM_CUSTOM_BTN_COUNT    2U      /**< 自定义按键数量                   */

/* ── 帧字节偏移 ── */
#define VTM_FRAME_DATA_OFFSET   2U      /**< 帧数据起始偏移（跳过 2 字节帧头）*/
#define VTM_CH0_BYTE            2U      /**< 通道 0 起始字节                  */
#define VTM_MODE_BYTE           7U      /**< 模式/按键复合字节                */
#define VTM_CUSTOM_BYTE         8U      /**< 自定义按键 / 拨盘字节            */
#define VTM_DIAL_H_BYTE         9U      /**< 拨盘高位字节                     */
#define VTM_MOUSE_X_L_BYTE      10U     /**< 鼠标 X 轴低字节                  */
#define VTM_MOUSE_BTN_BYTE      16U     /**< 鼠标按键字节                     */
#define VTM_KB_BYTE1            17U     /**< 键盘状态字节 1                   */
#define VTM_KB_BYTE2            18U     /**< 键盘状态字节 2                   */

/* ── 离线检测默认超时 ── */
#define VTM_DEFAULT_TIMEOUT_MS  500U    /**< 默认离线判定超时时间（ms）       */

/* ========================================================================== */
/*                              数据结构定义                                    */
/* ========================================================================== */

/**
 * @brief  VTM 视觉模块控制数据结构
 */
typedef __packed struct
{
    __packed struct
    {
        uint16_t channel[VTM_CHANNEL_COUNT]; /**< 摇杆通道值，中值 = 1024        */
        uint8_t  mode_switch;                /**< 模式切换拨杆                   */
        uint8_t  pause_button;               /**< 暂停按键                       */
        uint8_t  custom_button[VTM_CUSTOM_BTN_COUNT]; /**< 自定义按键 0/1        */
        uint16_t dial;                       /**< 拨盘值，中值 = 1024            */
        uint8_t  trigger;                    /**< 扳机键                         */
    } rc;

    __packed struct
    {
        int16_t x_axis;       /**< 鼠标 X 轴速度 */
        int16_t y_axis;       /**< 鼠标 Y 轴速度 */
        int16_t z_axis;       /**< 鼠标 Z 轴速度 */
        uint8_t left_button;  /**< 鼠标左键状态  */
        uint8_t right_button; /**< 鼠标右键状态  */
        uint8_t middle_button;/**< 鼠标中键状态  */
    } mouse;

    __packed struct
    {
        uint8_t w;     uint8_t s;     uint8_t a;     uint8_t d;
        uint8_t shift; uint8_t ctrl;  uint8_t q;     uint8_t e;
        uint8_t r;     uint8_t f;     uint8_t g;     uint8_t z;
        uint8_t x;     uint8_t c;     uint8_t v;     uint8_t b;
    } keyboard; /**< 各键单独 uint8_t，0=未按，1=已按 */

} VTM_ctrl_t;

/* ========================================================================== */
/*                          全局控制数据（控制层只读）                           */
/* ========================================================================== */

extern VTM_ctrl_t g_vtm_ctrl; /**< VTM 图传模块控制数据，控制层只读 */

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  检查 VTM 图传模块是否离线
 * @note   建议在 1ms 定时器中断或主循环中周期性调用。
 *         离线时自动将控制数据复位为安全默认值。
 * @param  timeout_ms  离线判定超时阈值（ms），推荐使用 VTM_DEFAULT_TIMEOUT_MS
 * @retval 1: 离线, 0: 在线
 */
uint8_t VTM_CheckOffline(uint32_t timeout_ms);

#endif /* VTM_DRIVER_H */
