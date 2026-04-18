/**
 ******************************************************************************
 * @file    bsp_can.c
 * @brief   CAN 总线底层驱动（BSP 层）
 * @note    职责边界：
 *            - 发送：封装 HAL CAN 发送接口
 *            - 接收：从 FIFO 取帧后，通过 __weak 回调通知上层
 *          【禁止】在本文件中 include 任何驱动层/控制层/应用层头文件
 * @version 1.1
 * @date    2026-4-4
 * @author  MOS
 ******************************************************************************
 */

#include "bsp_can.h"

/* ========================================================================== */
/*                      __weak 回调默认空实现                                   */
/* ========================================================================== */

/**
 * @brief  CAN 帧接收回调默认空实现（弱符号）
 *         驱动层在 motor_driver.c 中覆盖此函数以接入解析逻辑
 */
__weak void BSP_CAN_RxFrameCallback(CAN_HandleTypeDef *hcan,
                                     uint32_t           stdId,
                                     const uint8_t     *data)
{
    (void)hcan;
    (void)stdId;
    (void)data;
}

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  通过指定 CAN 总线发送一帧标准数据帧
 */
void BSP_CAN_TxData(CAN_HandleTypeDef *hcan, uint32_t stdId, uint8_t *data)
{
    uint32_t txMailbox;
    CAN_TxHeaderTypeDef txHeader = {
        .StdId = stdId,
        .IDE   = CAN_ID_STD,
        .RTR   = CAN_RTR_DATA,
        .DLC   = CAN_STD_DLC,
    };
    /* TODO: 可添加返回值校验与错误处理 */
    HAL_CAN_AddTxMessage(hcan, &txHeader, data, &txMailbox);
}

/* ========================================================================== */
/*                              HAL 回调实现                                    */
/* ========================================================================== */

/**
 * @brief  CAN FIFO0 接收中断回调（HAL 弱函数覆盖）
 * @note   BSP 层仅负责从 FIFO 读取原始帧，随后通过
 *         BSP_CAN_RxFrameCallback() 通知上层，本层不做任何解析。
 */
static uint32_t errorCount = 0; 
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[CAN_STD_DLC];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
    {
        errorCount++; /* 统计接收错误次数 */
        return;
    }

    /* 通过 weak 回调将原始帧上传至驱动层，BSP 不感知帧内容 */
    BSP_CAN_RxFrameCallback(hcan, rxHeader.StdId, rxData);
}
