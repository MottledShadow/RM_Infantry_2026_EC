/**
 ******************************************************************************
 * @file    rsi_driver.h
 * @brief   裁判系统（Referee System Interface）设备驱动头文件（设备驱动层）
 * @note    层级说明：
 *            本文件属于【设备驱动层】，位于 BSP 层之上、控制层之下。
 *            职责：覆盖 BSP_UART_RSI_DataReadyCallback()，
 *                  解析裁判系统串口帧，提取功率热量与机器人状态数据。
 *
 *          RM 裁判系统串口帧结构（详情请查看通信协议）：
 *            [SOF(1)] [data_len(2)] [seq(1)] [CRC8(1)] [cmd_id(2)] [data(N)] [CRC16(2)]
 *            Byte 0   Byte 1~2      Byte 3   Byte 4    Byte 5~6    Byte 7+
 *            → RSI_FRAME_HEADER_LEN = 7（SOF + data_len + seq + CRC8 + cmd_id）
 *
 * @version 1.0
 * @date    2026-4-4
 * @author  MOS
 ******************************************************************************
 */

#ifndef RSI_DRIVER_H
#define RSI_DRIVER_H

#include "main.h"
#include <stdint.h>
#include "bsp_uart.h"  /* 覆盖 __weak 回调，依赖方向正确（驱动层 → BSP 层）*/
#include <string.h>    /* memcpy */

/* ========================================================================== */
/*                              协议常量定义                                    */
/* ========================================================================== */

/* ── 帧结构偏移 ── */
#define RSI_FRAME_HEADER_LEN        7U  /**< 帧头长度（到数据段起始的偏移） */
#define RSI_CMD_ID_L_BYTE           5U  /**< cmd_id 低字节偏移              */
#define RSI_CMD_ID_H_BYTE           6U  /**< cmd_id 高字节偏移              */

/* ── cmd_id 定义 ── */
#define RSI_CMD_ROBOT_STATUS        0x0201U /**< 机器人状态数据帧 ID */
#define RSI_CMD_POWER_HEAT          0x0202U /**< 功率热量数据帧 ID   */

/* ── 各命令数据段长度 ── */
#define RSI_ROBOT_STATUS_DATA_LEN   13U /**< 机器人状态数据字节数    */
#define RSI_POWER_HEAT_DATA_LEN     14U /**< 功率热量数据字节数      */

/* ── robot_status 位域掩码 ── */
#define RSI_RS_GIMBAL_OUTPUT_MASK   0x01U /**< 云台电源输出位     */
#define RSI_RS_CHASSIS_OUTPUT_MASK  0x02U /**< 底盘电源输出位     */
#define RSI_RS_SHOOTER_OUTPUT_MASK  0x04U /**< 发射电源输出位     */

/* ── power_heat_data 字段字节偏移（相对于数据段起始）── */
#define RSI_PH_BUFFER_ENERGY_L      8U  /**< 缓冲能量低字节  */
#define RSI_PH_BUFFER_ENERGY_H      9U  /**< 缓冲能量高字节  */
#define RSI_PH_HEAT_17MM_L          10U /**< 17mm 热量低字节 */
#define RSI_PH_HEAT_17MM_H          11U /**< 17mm 热量高字节 */
#define RSI_PH_HEAT_42MM_L          12U /**< 42mm 热量低字节 */
#define RSI_PH_HEAT_42MM_H          13U /**< 42mm 热量高字节 */

/* ========================================================================== */
/*                              数据结构定义                                    */
/* ========================================================================== */

/**
 * @brief  机器人状态数据（对应 cmd_id = 0x0201）
 * @note   power_management_* 使用位域，由解析函数手动赋值（逐位提取）
 */
typedef __packed struct
{
    uint8_t  robot_id;                          /**< 本机器人ID              */
    uint8_t  robot_level;                       /**< 机器人等级              */
    uint16_t current_HP;                        /**< 当前血量                */
    uint16_t maximum_HP;                        /**< 血量上限                */
    uint16_t shooter_barrel_cooling_value;      /**< 枪管冷却速率（/s）      */
    uint16_t shooter_barrel_heat_limit;         /**< 枪管热量上限            */
    uint16_t chassis_power_limit;               /**< 底盘功率上限（W）       */
    uint8_t  power_management_gimbal_output :1; /**< 云台电源输出使能，1为输出*/
    uint8_t  power_management_chassis_output:1; /**< 底盘电源输出使能，1为输出*/
    uint8_t  power_management_shooter_output:1; /**< 发射电源输出使能，1为输出*/
} RSI_RobotStatus_t;

/**
 * @brief  功率与热量数据（对应 cmd_id = 0x0202）
 * @note   字段偏移基于 RM 2026 通信协议，字节 0~7 为保留位（未解析）
 */
typedef __packed struct
{
    uint16_t buffer_energy;              /**< 缓冲能量，单位 J          */
    uint16_t shooter_17mm_barrel_heat;   /**< 17mm 枪管热量             */
    uint16_t shooter_42mm_barrel_heat;   /**< 42mm 枪管热量             */
} RSI_PowerHeat_t;

/* ========================================================================== */
/*                          全局数据（控制层只读）                               */
/* ========================================================================== */

extern RSI_PowerHeat_t   g_rsi_power_heat;  /**< 功率热量数据，控制层只读 */
extern RSI_RobotStatus_t g_rsi_robot_status; /**< 机器人状态，控制层只读  */

#endif /* RSI_DRIVER_H */
