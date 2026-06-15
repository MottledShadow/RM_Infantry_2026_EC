/**
 ******************************************************************************
 * @file    referee_protocol.h
 * @brief   裁判系统 UI 通信协议封装头文件（设备驱动层）
 * @note    层级说明：
 *            本文件属于【设备驱动层】，位于 BSP 层之上、应用层之下。
 *            职责：将应用层的图形数据按 RM 裁判系统串口协议打包，
 *                  并通过 BSP_UART_TxData() 发送。
 *            依赖链：referee_protocol → crc（算法层）→ bsp_uart（BSP 层）
 *
 *          RM 裁判系统自定义图形协议帧结构：
 *            [SOF(1)] [data_len(2,LE)] [seq(1)] [CRC8(1)]
 *            [cmd_id(2,LE)] [data_cmd_id(2)] [sender_id(2)] [receiver_id(2)]
 *            [user_data(N)] [CRC16(2,LE)]
 *            ↑────────── 固定帧头开销 = REFEREE_FRAME_OVERHEAD 字节 ──────────↑
 *
 *          客户端 ID 规则：receiver_id = 0x0100 + robot_id
 *
 * @version 1.0
 * @date    2026-4-6
 * @author  MOS
 ******************************************************************************
 */

#ifndef REFEREE_PROTOCOL_H
#define REFEREE_PROTOCOL_H

#include "main.h"
#include <string.h>
#include <stdint.h>
#include "crc.h"         /* 算法层：CRC8 / CRC16 计算  */
#include "bsp_uart.h"    /* BSP 层：UART DMA 发送       */

/* ========================================================================== */
/*                          协议固定字段定义                                    */
/* ========================================================================== */

#define REFEREE_FRAME_SOF           0xA5U   /**< 帧起始字节（Start of Frame）*/
#define REFEREE_MAIN_CMD_ID         0x0301U /**< 机器人交互数据主命令 ID      */
#define REFEREE_CLIENT_ID_OFFSET    0x0100U /**< 客户端 ID = 此偏移 + robot_id*/

/**
 * @brief  帧固定开销字节数
 * @note   = SOF(1) + data_len(2) + seq(1) + CRC8(1) + cmd_id(2)
 *           + data_cmd_id(2) + sender_id(2) + receiver_id(2) + CRC16(2)
 *         = 15 字节（不含 user_data）
 */
#define REFEREE_FRAME_OVERHEAD      15U

/** 发送缓冲区最大长度（裁判系统单帧上限）*/
#define REFEREE_TX_BUF_SIZE         127U

/* ========================================================================== */
/*                          子命令 ID（图形数据包类型）                          */
/* ========================================================================== */

#define REFEREE_SUB_CMD_DRAW_1      0x0101U /**< 绘制 1 个图形 */
#define REFEREE_SUB_CMD_DRAW_2      0x0102U /**< 绘制 2 个图形 */
#define REFEREE_SUB_CMD_DRAW_5      0x0103U /**< 绘制 5 个图形 */
#define REFEREE_SUB_CMD_DRAW_7      0x0104U /**< 绘制 7 个图形 */
#define REFEREE_SUB_CMD_DRAW_CHAR   0x0110U /**< 绘制字符串    */

/* ========================================================================== */
/*                              数据结构定义                                    */
/* ========================================================================== */

/**
 * @brief  机器人交互数据头（跟在 cmd_id = 0x0301 帧中）
 */
typedef __packed struct
{
    uint16_t data_cmd_id;   /**< 子命令 ID，决定 user_data 内容类型 */
    uint16_t sender_id;     /**< 发送方机器人 ID                    */
    uint16_t receiver_id;   /**< 接收方 ID（通常为客户端 0x0100+id）*/
} RefereeInteractionHeader_t;

/**
 * @brief  单个图形描述符（RM 裁判系统协议 Figure 结构）
 * @note   字段含义见 RM 裁判系统开发手册。
 *         operate_tpye / figure_tpye 为 DJI 原始拼写，保留以兼容协议文档。
 */
typedef __packed struct
{
    uint8_t  figure_name[3];       /**< 图形名称（3字节，用于增删改识别）*/
    uint32_t operate_tpye   : 3;   /**< 操作类型：0=空，1=增，2=改，3=删  */
    uint32_t figure_tpye    : 3;   /**< 图形类型：0=线，1=矩形，2=圆 ...  */
    uint32_t layer          : 4;   /**< 图层（0~9）                        */
    uint32_t color          : 4;   /**< 颜色（0=队伍色，1=黄，2=绿...）   */
    uint32_t details_a      : 9;   /**< 起始角度 / 字号（依图形类型而定） */
    uint32_t details_b      : 9;   /**< 终止角度 / 字符长度               */
    uint32_t width          : 10;  /**< 线宽                               */
    uint32_t start_x        : 11;  /**< 起点 X（像素，左下角为原点）       */
    uint32_t start_y        : 11;  /**< 起点 Y                             */
    uint32_t details_c      : 10;  /**< 半径 / 字体大小（依图形类型）     */
    uint32_t details_d      : 11;  /**< 终点 X / 圆心 X                   */
    uint32_t details_e      : 11;  /**< 终点 Y / 圆心 Y                   */
} RefereeUIFigure_t;

/**
 * @brief  5 图形数据包（对应子命令 0x0103）
 * @note   原名 interaction_figure_3_t，其实包含 5 个图形，命名有误，已修正。
 */
typedef __packed struct
{
    RefereeUIFigure_t figure[5];
} RefereeUIFigure5_t;

/**
 * @brief  2 图形数据包（对应子命令 0x0102）
 */
typedef __packed struct
{
    RefereeUIFigure_t figure[2];
} RefereeUIFigure2_t;

/**
 * @brief  字符串数据包（对应子命令 0x0110）
 */
typedef __packed struct
{
    RefereeUIFigure_t figure;   /**< 文字图形描述符 */
    uint8_t           text[30]; /**< 字符串内容，最多 30 字节 */
} RefereeUIChar_t;

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  打包并发送裁判系统自定义图形数据帧
 * @note   内部自动完成：
 *           1. 组装帧头（SOF / data_len / seq=0 / CRC8）
 *           2. 填写 cmd_id（0x0301）与交互数据头（sub_cmd_id / sender / receiver）
 *           3. 复制 user_data
 *           4. 追加 CRC16
 *           5. 调用 BSP_UART_TxData() 发送
 * @param  sub_cmd_id   子命令 ID，决定图形包类型（见 REFEREE_SUB_CMD_* 宏）
 * @param  robot_id     本机机器人 ID（由裁判系统分配）
 * @param  user_data    图形数据指针（指向 RefereeUIFigureX_t 结构体）
 * @param  data_length  user_data 的字节数（sizeof(RefereeUIFigureX_t)）
 * @param  huart        目标 UART 句柄指针（通常为 &huart6）
 * @retval 无
 */
void Referee_SendUIData(uint16_t          sub_cmd_id,
                        uint16_t          robot_id,
                        const uint8_t    *user_data,
                        uint8_t           data_length,
                        UART_HandleTypeDef *huart);

#endif /* REFEREE_PROTOCOL_H */
