/**
 ******************************************************************************
 * @file    referee_ui.c
 * @brief   裁判系统自定义 UI 绘制实现（应用层）
 * @note    职责边界：
 *            定义图形元素的业务参数，调用 Referee_SendUIData() 打包发送。
 *            【禁止】在本文件中实现协议封装或 CRC 计算。
 *
 * @version 1.0
 * @date    2026-4-6
 * @author  MOS
 ******************************************************************************
 */

#include "referee_ui.h"

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  绘制瞄准点 UI 元素（矩形框）
 */
void UI_DrawAimPoint(void)
{
    RefereeUIFigure5_t fig_pkg;
    memset(&fig_pkg, 0, sizeof(fig_pkg));

    /* 填写第 0 个图形：瞄准点矩形框 */
    fig_pkg.figure[0].figure_name[0] = AIM_POINT_NAME_0;
    fig_pkg.figure[0].figure_name[1] = AIM_POINT_NAME_1;
    fig_pkg.figure[0].figure_name[2] = AIM_POINT_NAME_2;

    fig_pkg.figure[0].operate_tpye = AIM_POINT_OPERATE;
    fig_pkg.figure[0].figure_tpye  = AIM_POINT_TYPE;
    fig_pkg.figure[0].layer        = AIM_POINT_LAYER;
    fig_pkg.figure[0].color        = AIM_POINT_COLOR;
    fig_pkg.figure[0].width        = AIM_POINT_WIDTH;
    fig_pkg.figure[0].start_x      = AIM_POINT_START_X;
    fig_pkg.figure[0].start_y      = AIM_POINT_START_Y;
    fig_pkg.figure[0].details_d    = AIM_POINT_END_X;
    fig_pkg.figure[0].details_e    = AIM_POINT_END_Y;
    /* 图形 1~4 保持全零（memset 已清零）*/

    Referee_SendUIData(REFEREE_SUB_CMD_DRAW_5,
                       g_rsi_robot_status.robot_id,
                       (const uint8_t *)&fig_pkg,
                       AIM_POINT_DATA_LEN,
                       REFEREE_UI_HUART);
}
