/**
 ******************************************************************************
 * @file    filter.c
 * @brief   CAN 总线过滤器初始化实现（BSP 层）
 * @note    职责边界：
 *            配置并激活 CAN1 / CAN2 的接收过滤器，
 *            完成后两条总线均处于运行状态并开启 FIFO0 接收中断。
 *
 *          过滤器配置说明：
 *            FilterBank 0   → CAN1，掩码全 0 = 接收所有帧
 *            FilterBank 14  → CAN2（从过滤器起始 = 14），掩码全 0 = 接收所有帧
 *            FilterFIFO     → FIFO0，触发 RxFifo0MsgPendingCallback
 *            FilterMode     → IDMASK（掩码模式，比列表模式更灵活）
 *            FilterScale    → 32BIT（使用单个 32 位过滤器）
 *
 * @version 1.0
 * @date    2022-9-3
 * @author  DJI
 ******************************************************************************
 */

#include "filter.h"

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
