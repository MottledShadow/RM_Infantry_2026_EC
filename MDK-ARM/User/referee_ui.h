/**
 ******************************************************************************
 * @file    referee_ui.h
 * @brief   裁判系统自定义 UI 绘制头文件（应用层）
 * @note    层级说明：
 *            本文件属于【应用层】，是整个工程中 UI 业务逻辑的顶层。
 *            职责：定义图形元素的业务参数（坐标、颜色、尺寸），
 *                  调用 referee_protocol 层打包发送，不关心协议细节。
 *            依赖链：
 *              referee_ui → referee_protocol（设备驱动层）
 *              referee_ui → rsi_driver（设备驱动层，读 g_rsi_robot_status.robot_id）
 *
 *          UI 坐标系说明：
 *            原点在屏幕左下角，X 轴向右，Y 轴向上。
 *            分辨率：1920 × 1080
 *
 * @version 1.0
 * @date    2026-4-6
 * @author  MOS
 ******************************************************************************
 */

#ifndef REFEREE_UI_H
#define REFEREE_UI_H

#include "referee_protocol.h"
#include "rsi_driver.h"  /* 读取 g_rsi_robot_status.robot_id */
#include "main.h"
#include <string.h>  /* memset */

/* ========================================================================== */
/*                          UART 端口配置                                       */
/* ========================================================================== */

/**
 * @brief  裁判系统 UART 端口（连接裁判系统主控的串口）
 * @note   TODO: 根据硬件连接确认端口号。
 */
#define REFEREE_UI_HUART    (&huart6)

/* ========================================================================== */
/*                          瞄准点图形参数                                      */
/* ========================================================================== */

/** 瞄准点图形名称（3 字节唯一标识，用于协议层识别/增删改）*/
#define AIM_POINT_NAME_0    'A'
#define AIM_POINT_NAME_1    'I'
#define AIM_POINT_NAME_2    'M'

#define AIM_POINT_OPERATE   1U  /**< 操作类型：1=新增（首次发送）             */
#define AIM_POINT_TYPE      1U  /**< 图形类型：1=矩形                         */
#define AIM_POINT_LAYER     0U  /**< 图层：0                                  */
#define AIM_POINT_COLOR     2U  /**< 颜色：2=绿色                             */
#define AIM_POINT_WIDTH     3U  /**< 线宽（像素）                             */

/** 瞄准点矩形左下角坐标（像素）TODO: 根据实际瞄准标定值调整 */
#define AIM_POINT_START_X   950U
#define AIM_POINT_START_Y   465U

/** 瞄准点矩形右上角坐标（像素）TODO: 根据实际瞄准标定值调整 */
#define AIM_POINT_END_X     970U
#define AIM_POINT_END_Y     485U

/** 5 图形包数据长度（sizeof(RefereeUIFigure5_t)）*/
#define AIM_POINT_DATA_LEN  75U

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  绘制瞄准点 UI 元素（矩形框）
 * @note   使用 5 图形包协议，仅填充第 0 个图形，其余留空。
 *         robot_id 从 g_rsi_robot_status.robot_id 自动读取。
 * @retval 无
 */
void UI_DrawAimPoint(void);

#endif /* REFEREE_UI_H */
