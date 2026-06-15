/**
 ******************************************************************************
 * @file    rsi_driver.c
 * @brief   裁判系统设备驱动实现（设备驱动层）
 * @note    职责边界：
 *            - 覆盖 BSP_UART_RSI_DataReadyCallback() 接入原始数据
 *            - 解析 cmd_id，分发到对应的数据提取函数
 *            - 维护功率热量与机器人状态全局结构体
 *          【禁止】在本文件中调用控制层或应用层接口
 *
 *          ⚠️  当前仅解析 0x0201 / 0x0202 两个 cmd_id，
 *              如需扩展其他裁判系统数据包，在 RSI_DispatchFrame() 的
 *              switch 中添加对应 case 即可。
 * @version 1.0
 * @date    2026-4-4
 * @author  MOS
 ******************************************************************************
 */

#include "rsi_driver.h"

/* ========================================================================== */
/*                          全局裁判系统数据                                    */
/* ========================================================================== */

RSI_PowerHeat_t   g_rsi_power_heat;
RSI_RobotStatus_t g_rsi_robot_status;

/* ========================================================================== */
/*                              内部函数                                        */
/* ========================================================================== */

/**
 * @brief  解析功率热量数据段（cmd_id = RSI_CMD_POWER_HEAT）
 * @param  data  指向数据段起始的指针（不含帧头，不含 cmd_id）
 * @note   data[0~7] 为协议中保留位
 */
static void RSI_ParsePowerHeat(const uint8_t *data)
{
    g_rsi_power_heat.buffer_energy =
        (uint16_t)(data[RSI_PH_BUFFER_ENERGY_L] | ((uint16_t)data[RSI_PH_BUFFER_ENERGY_H] << 8));
    g_rsi_power_heat.shooter_17mm_barrel_heat =
        (uint16_t)(data[RSI_PH_HEAT_17MM_L] | ((uint16_t)data[RSI_PH_HEAT_17MM_H] << 8));
    g_rsi_power_heat.shooter_42mm_barrel_heat =
        (uint16_t)(data[RSI_PH_HEAT_42MM_L] | ((uint16_t)data[RSI_PH_HEAT_42MM_H] << 8));
}

/**
 * @brief  解析机器人状态数据段（cmd_id = RSI_CMD_ROBOT_STATUS）
 * @param  data  指向数据段起始的指针（不含帧头，不含 cmd_id）
 */
static void RSI_ParseRobotStatus(const uint8_t *data)
{
    g_rsi_robot_status.robot_id                    = data[0];
    g_rsi_robot_status.robot_level                 = data[1];
    g_rsi_robot_status.current_HP                  = (uint16_t)(data[2] | ((uint16_t)data[3]  << 8));
    g_rsi_robot_status.maximum_HP                  = (uint16_t)(data[4] | ((uint16_t)data[5]  << 8));
    g_rsi_robot_status.shooter_barrel_cooling_value= (uint16_t)(data[6] | ((uint16_t)data[7]  << 8));
    g_rsi_robot_status.shooter_barrel_heat_limit   = (uint16_t)(data[8] | ((uint16_t)data[9]  << 8));
    g_rsi_robot_status.chassis_power_limit         = (uint16_t)(data[10]| ((uint16_t)data[11] << 8));

    /* 位域手动提取（协议 data[12] 低 3 位）*/
    g_rsi_robot_status.power_management_gimbal_output  = (data[12] & RSI_RS_GIMBAL_OUTPUT_MASK)  ? 1U : 0U;
    g_rsi_robot_status.power_management_chassis_output = (data[12] & RSI_RS_CHASSIS_OUTPUT_MASK) ? 1U : 0U;
    g_rsi_robot_status.power_management_shooter_output = (data[12] & RSI_RS_SHOOTER_OUTPUT_MASK) ? 1U : 0U;
}

/**
 * @brief  根据 cmd_id 分发帧数据到对应解析函数
 * @param  frame  指向完整帧缓冲区起始的指针
 */
static void RSI_DispatchFrame(const uint8_t *frame)
{
    /* 从固定偏移提取 cmd_id（小端 16 位）*/
    uint16_t cmd_id = (uint16_t)(frame[RSI_CMD_ID_L_BYTE]
                    | ((uint16_t)frame[RSI_CMD_ID_H_BYTE] << 8));

    /* 数据段起始指针（跳过帧头与 cmd_id）*/
    const uint8_t *data = frame + RSI_FRAME_HEADER_LEN;

    switch (cmd_id)
    {
        case RSI_CMD_ROBOT_STATUS:
            RSI_ParseRobotStatus(data);
            break;

        case RSI_CMD_POWER_HEAT:
            RSI_ParsePowerHeat(data);
            break;

        default:
            break;
    }
}

/* ========================================================================== */
/*                    覆盖 BSP_UART __weak 回调                                 */
/* ========================================================================== */

/**
 * @brief  覆盖 bsp_uart.c 中的弱符号回调，接入裁判系统帧解析
 * @note   优化方向: 建议在此处添加帧头（0xA5）校验与 CRC 校验，
 *                  提高协议鲁棒性，当前版本跳过校验直接解析。
 */
void BSP_UART_RSI_DataReadyCallback(uint8_t *buf, uint16_t size)
{
    if (size < RSI_FRAME_HEADER_LEN)
        return; /* 帧长不足，丢弃 */

    RSI_DispatchFrame(buf);
}
