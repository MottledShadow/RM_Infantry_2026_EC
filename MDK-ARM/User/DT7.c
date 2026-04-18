/**
 ******************************************************************************
 * @file    DT7.c
 * @brief   DT7 遥控器应用控制实现（应用层）
 * @note    职责边界：
 *            解析 DT7 拨杆状态，调用控制层接口，不包含任何控制算法。
 *            【禁止】在本文件中实现 PID / 状态机 / 硬件驱动等逻辑。
 *
 * @version 1.0
 * @date    2026-4-6
 * @author  MOS
 ******************************************************************************
 */

#include "DT7.h"

/* ========================================================================== */
/*                          模块私有状态                                        */
/* ========================================================================== */

/**
 * @brief  底盘旋转速度（静态，Q/E 键控制或随动模式共用）
 * @note   Q/E 模式：每帧线性增减直到 ±VTM_MAX_ROTATIONAL_SPEED
 *         随动模式：由 Yaw 角误差实时计算
 */
static float s_rotational_velocity = 0.0f;

/* ========================================================================== */
/*                          私有辅助函数                                        */
/* ========================================================================== */

/**
 * @brief  摇杆模式下的底盘旋转速度计算（拨盘输入）
 * @note   双模式策略，以拨盘中值（1024）为分界：
 *
 *         拨盘居中（digl == 1024）→ 底盘随云台随动模式
 *           旋转速度 = -yaw_angle × VTM_YAW_FOLLOW_GAIN
 *           yaw_angle 在死区内（< VTM_YAW_DEADBAND）时归零，防止抖动。
 *
 *         拨盘偏转（digl != 1024）→ 直接速度控制模式
 *           旋转速度 = digl - 1024（有符号偏差量）
 *           拨盘向上（digl > 1024）为正值（逆时针），反之为负值（顺时针）。
 *
 *         digl == 0 时函数直接返回，不更新 s_rotational_velocity，
 *         保持上一帧的旋转速度。
 *
 * @param  digl       VTM 拨盘通道原始值（范围 [0, 2047]，中值 = 1024）
 * @param  yaw_angle  云台相对底盘的 Yaw 偏转角（rad），来自 Gimbal_GetYawRelativeAngle()
 * @retval 无（结果写入模块静态变量 s_rotational_velocity）
 */
static void RC_ComputeRotational(uint16_t digl, float yaw_angle)
{
    if (digl == 0U) {
        s_rotational_velocity = 0.0f;  // 更安全的选择
        return;
    }
    if (digl > 0U)
    {
        if (digl == 1024U)
        {
            /* 拨盘居中：底盘随云台随动
             * yaw_angle 在死区内视为已对齐，避免微小误差引起持续抖动 */
            if (fabsf(yaw_angle) < DT7_YAW_DEADBAND)
                s_rotational_velocity = 0.0f;
            else
                s_rotational_velocity = -yaw_angle * DT7_YAW_FOLLOW_GAIN;
        }
        else
        {
            /* 拨盘偏转：直接以偏差量作为旋转速度指令
             * 偏差量范围：[-1024, +1023]，单位与速度环 PID 设定值一致 */
            s_rotational_velocity = (float)((int16_t)digl - 1024);
        }
    }
}

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  DT7 遥控器应用控制函数
 */
void DT7_Control(DT7_ctrl_t *dt7_ctrl)
{
    /* 获取本帧 Yaw 相对角（供底盘坐标变换使用）*/
    float yaw_rel = Gimbal_GetYawRelativeAngle();

    switch (dt7_ctrl->rc.s[0])
    {
        /* ── 上方：全功能运行模式 ── */
        case DT7_SWITCH_UP:
        {
            /* 计算旋转速度（拨盘输入）*/
            RC_ComputeRotational(dt7_ctrl->rc.ch[4], yaw_rel);

            /* 底盘：左摇杆控制平移，无旋转指令（随动旋转由底盘模块内部处理）*/
            Chassis_Control((float)DT7_CH_OFFSET(dt7_ctrl, DT7_CH_CHASSIS_X),
                            (float)-DT7_CH_OFFSET(dt7_ctrl, DT7_CH_CHASSIS_Y),
                            s_rotational_velocity,
                            yaw_rel);

            /* 云台：右摇杆控制 Yaw / Pitch 角速度 */
            Gimbal_Control((float)-DT7_CH_OFFSET(dt7_ctrl, DT7_CH_YAW)   * DT7_GIMBAL_SENSITIVITY,
                           (float) DT7_CH_OFFSET(dt7_ctrl, DT7_CH_PITCH) * DT7_GIMBAL_SENSITIVITY);

            /* 摩擦轮：全功能模式下启动发射转速 */
            FrictionWheel_SetFire();

            /* 右拨杆上方：调试模式，直接向 M2006 发送固定转矩（绕过热量控制）*/
            if (dt7_ctrl->rc.s[1] == DT7_SWITCH_UP)
            {
                /*
                 * ⚠️  此为硬件调试路径，直接发送固定电流指令。
                 *     正式比赛时应通过 Shooter_Control() 控制拨弹盘。
                 */
                uint8_t buf[8] = {0U};
                buf[0] = (uint8_t)((uint16_t)DT7_DEBUG_M2006_TORQUE >> 8U);
                buf[1] = (uint8_t)((uint16_t)DT7_DEBUG_M2006_TORQUE & 0xFFU);
                BSP_CAN_TxData(SHOOTER_CAN_HANDLE, M2006_M3508_CTRL_ID_GROUP1, buf);
            }
            else
            {
                uint8_t buf[8] = {0U};
                BSP_CAN_TxData(SHOOTER_CAN_HANDLE, M2006_M3508_CTRL_ID_GROUP1, buf);
            }
            break;
        }

        /* ── 中间：运动控制模式（摩擦轮不启动）── */
        case DT7_SWITCH_MID:
        {
            /* 计算旋转速度（拨盘输入）*/
            RC_ComputeRotational(dt7_ctrl->rc.ch[4], yaw_rel);
        
            Chassis_Control((float)DT7_CH_OFFSET(dt7_ctrl, DT7_CH_CHASSIS_X),
                            (float)-DT7_CH_OFFSET(dt7_ctrl, DT7_CH_CHASSIS_Y),
                            s_rotational_velocity,
                            yaw_rel);

            Gimbal_Control((float)-DT7_CH_OFFSET(dt7_ctrl, DT7_CH_YAW)   * DT7_GIMBAL_SENSITIVITY,
                           (float) DT7_CH_OFFSET(dt7_ctrl, DT7_CH_PITCH) * DT7_GIMBAL_SENSITIVITY);
            FrictionWheel_SetIdle();
            break;
        }

        /* ── 下方：安全停机模式 ── */
        case DT7_SWITCH_DOWN:
        {
            /*
             * 向所有电机发送零转矩帧，强制停机。
             * 此处直接调用 BSP_CAN_TxData 而非控制层接口，
             * 是因为需要绕过控制层状态机立即停机（紧急停止语义）。
             */
            uint8_t zero[8] = {0U};

            /* 底盘 3508 清零（hcan1, 0x200）*/
            BSP_CAN_TxData(CHASSIS_CAN_HANDLE, M2006_M3508_CTRL_ID_GROUP1, zero);

            /* 云台 6020 清零
             * TODO: 确认 DT7_GIMBAL_DISABLE_CAN_ID (0x1FE) 是否应为 GIMBAL_CAN_TX_ID (0x1FF) */
            BSP_CAN_TxData(GIMBAL_CAN_HANDLE, GM6020_CTRL_ID_GROUP1, zero);

            /* 拨弹盘 M2006 清零（hcan2, 0x200）*/
            BSP_CAN_TxData(SHOOTER_CAN_HANDLE, M2006_M3508_CTRL_ID_GROUP1, zero);

            /* 摩擦轮回到待机脉宽 */
            FrictionWheel_SetIdle();
            break;
        }

        default:
            break;
    }
}
