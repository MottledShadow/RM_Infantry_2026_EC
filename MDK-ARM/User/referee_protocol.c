/**
 ******************************************************************************
 * @file    referee_protocol.c
 * @brief   裁判系统 UI 通信协议封装实现（设备驱动层）
 * @note    职责边界：
 *            按协议格式打包帧并通过 BSP_UART_TxData() 发送，
 *            不包含任何图形业务逻辑。
 *            【禁止】直接调用 HAL_UART_*，统一由 BSP_UART_TxData() 代理。
 *
 * @version 1.0
 * @date    2026-4-6
 * @author  MOS
 ******************************************************************************
 */

#include "referee_protocol.h"

/* ========================================================================== */
/*                          模块私有发送缓冲区                                  */
/* ========================================================================== */

/**
 * @brief  发送缓冲区（static，仅本模块可见）
 * @note   原版在文件作用域定义为全局变量 tx_buf，
 *         同时 uart_to_RSI 形参也叫 tx_buf，造成遮蔽，改为 static 消除歧义。
 *         裁判系统每次调用结束前 DMA 已启动，下次调用前帧已发完，
 *         裸机单任务场景下复用同一缓冲区是安全的。
 *         若将来需要多帧排队，需改为队列缓冲。
 */
static uint8_t s_tx_buf[REFEREE_TX_BUF_SIZE];

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  打包并发送裁判系统自定义图形数据帧
 */
void Referee_SendUIData(uint16_t           sub_cmd_id,
                        uint16_t           robot_id,
                        const uint8_t     *user_data,
                        uint8_t            data_length,
                        UART_HandleTypeDef *huart)
{
    uint16_t client_id   = REFEREE_CLIENT_ID_OFFSET + robot_id;
    uint16_t data_length_field = (uint16_t)sizeof(RefereeInteractionHeader_t)
                                 + (uint16_t)data_length;
    uint16_t total_length = data_length + REFEREE_FRAME_OVERHEAD;

    /* ── 帧头（Byte 0~3）── */
    s_tx_buf[0] = REFEREE_FRAME_SOF;
    s_tx_buf[1] = (uint8_t)(data_length_field & 0xFFU);         /* data_len 低字节 */
    s_tx_buf[2] = (uint8_t)((data_length_field >> 8U) & 0xFFU); /* data_len 高字节 */
    s_tx_buf[3] = 0U;                                            /* seq（固定为 0，可按需递增）*/

    /* ── CRC8（Byte 4，覆盖 Byte 0~3）── */
    s_tx_buf[4] = CRC8_Calc(s_tx_buf, 4U, CRC8_INIT_VALUE);

    /* ── 主命令 ID（Byte 5~6，小端）── */
    s_tx_buf[5] = (uint8_t)(REFEREE_MAIN_CMD_ID & 0xFFU);
    s_tx_buf[6] = (uint8_t)((REFEREE_MAIN_CMD_ID >> 8U) & 0xFFU);

    /* ── 交互数据头（Byte 7~12）── */
    s_tx_buf[7]  = (uint8_t)(sub_cmd_id & 0xFFU);       /* 子命令 ID 低字节 */
    s_tx_buf[8]  = (uint8_t)((sub_cmd_id >> 8U) & 0xFFU);
    s_tx_buf[9]  = (uint8_t)(robot_id & 0xFFU);          /* 发送者 ID 低字节 */
    s_tx_buf[10] = (uint8_t)((robot_id >> 8U) & 0xFFU);
    s_tx_buf[11] = (uint8_t)(client_id & 0xFFU);          /* 接收者 ID 低字节 */
    s_tx_buf[12] = (uint8_t)((client_id >> 8U) & 0xFFU);

    /* ── 用户数据（Byte 13 ~ 13+data_length-1）── */
    for (uint8_t i = 0U; i < data_length; i++)
    {
        s_tx_buf[13U + i] = user_data[i];
    }

    /* ── CRC16（末尾 2 字节，覆盖整帧 data_length+13 字节）── */
    CRC16_Append(s_tx_buf, (uint32_t)total_length);

    /* ── 发送 ── */
    BSP_UART_TxData(huart, s_tx_buf, total_length);
}
