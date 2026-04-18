/**
 ******************************************************************************
 * @file    VTM.c
 * @brief   VTM 图传模块应用控制实现（应用层）
 * @note    职责边界：
 *            解析键鼠/摇杆输入，调用控制层接口，不包含任何控制算法。
 *            【禁止】在本文件中实现 PID / 状态机 / 硬件驱动等逻辑。
 *
 * @version 1.0
 * @date    2026-4-6
 * @author  MOS
 ******************************************************************************
 */

#include "VTM.h"
#include "bsp_can.h"     /* BSP 层：安全停机直接发零帧                     */
#include "friction_wheel.h" /* 控制层：摩擦轮停机                           */
#include <math.h>        /* fabsf, sqrtf                                   */

/* ========================================================================== */
/*                          模块私有状态                                        */
/* ========================================================================== */

/**
 * @brief  底盘纵向速度（静态，实现键盘按下时的平滑加速）
 * @note   W/S 键每帧增减 VTM_TRANSLATIONAL_ACCEL，双键同按或均不按时清零。
 */
static float s_longitudinal_velocity = 0.0f;

/**
 * @brief  底盘侧向速度（静态，同上）
 * @note   A/D 键每帧增减 VTM_TRANSLATIONAL_ACCEL。
 */
static float s_lateral_velocity = 0.0f;

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
 * @brief  计算底盘平移速度（W/S/A/D 键，带加速度限制）
 * @note   每帧调用，根据按键状态增减速度并在最大值处饱和。
 *         双键同按视为中立（松手），避免误操作。
 * @param  vtm_ctrl  VTM 控制数据指针
 */
static void VTM_ComputeTranslational(const VTM_ctrl_t *vtm_ctrl)
{
    /* ── 纵向（W/S）── */
    uint8_t w = vtm_ctrl->keyboard.w;
    uint8_t s = vtm_ctrl->keyboard.s;

    if ((w == s))  /* 均未按或同时按 → 停止 */
    {
        s_longitudinal_velocity = 0.0f;
    }
    else if (w && s_longitudinal_velocity < VTM_MAX_TRANSLATIONAL_SPEED)
    {
        s_longitudinal_velocity += VTM_TRANSLATIONAL_ACCEL;
    }
    else if (s && s_longitudinal_velocity > -VTM_MAX_TRANSLATIONAL_SPEED)
    {
        s_longitudinal_velocity -= VTM_TRANSLATIONAL_ACCEL;
    }

    /* ── 侧向（A/D）── */
    uint8_t a = vtm_ctrl->keyboard.a;
    uint8_t d = vtm_ctrl->keyboard.d;

    if ((a == d))
    {
        s_lateral_velocity = 0.0f;
    }
    else if (a && s_lateral_velocity < VTM_MAX_TRANSLATIONAL_SPEED)
    {
        s_lateral_velocity += VTM_TRANSLATIONAL_ACCEL;
    }
    else if (d && s_lateral_velocity > -VTM_MAX_TRANSLATIONAL_SPEED)
    {
        s_lateral_velocity -= VTM_TRANSLATIONAL_ACCEL;
    }

    /* ── 合速度限幅（保持方向不变，幅值缩放到圆形边界）── */
    float speed_sq = s_longitudinal_velocity * s_longitudinal_velocity
                   + s_lateral_velocity      * s_lateral_velocity;
    if (speed_sq > VTM_MAX_TRANSLATIONAL_SPEED * VTM_MAX_TRANSLATIONAL_SPEED)
    {
        float scale = VTM_MAX_TRANSLATIONAL_SPEED / sqrtf(speed_sq);
        s_longitudinal_velocity *= scale;
        s_lateral_velocity      *= scale;
    }
}

/**
 * @brief  键鼠模式下的底盘旋转速度计算（Q/E 键点按切换小陀螺）
 * @note   使用 Key_Scan Toggle 型状态机处理 Q/E 键：
 *           每次完整按下+松开翻转对应的 CCW_state / CW_state（0→1 或 1→0）。
 *
 *         三态状态机（state 变量）：
 *           state 0（随动）：底盘跟随云台偏转角，yaw 在死区内时归零
 *           state 1（逆时针小陀螺）：每帧累加 VTM_ROTATION_ACCEL，饱和于 +MAX
 *           state 2（顺时针小陀螺）：每帧递减 VTM_ROTATION_ACCEL，饱和于 -MAX
 *
 *         状态切换规则（点按触发，不需要持续按住）：
 *           state 0 → 1：点按 Q（CCW_state 翻转为 1）
 *           state 0 → 2：点按 E（CW_state  翻转为 1）
 *           state 1 → 0：再次点按 Q（CCW_state 翻转回 0）
 *           state 1 → 2：点按 E（CW_state 翻转为 1），同时强制重置 CCW_state=0，
 *                         使下次 Q 键需要重新点按才能激活逆时针
 *           state 2 → 0：再次点按 E（CW_state 翻转回 0）
 *           state 2 → 1：点按 Q（CCW_state 翻转为 1），同时强制重置 CW_state=0
 *
 * @param  q_key  Q 键当前电平（1=按下，0=松开），由调用方从 vtm_ctrl->keyboard 传入
 * @param  e_key  E 键当前电平
 * @retval 无（结果写入模块静态变量 s_rotational_velocity）
 */
static void Keyboard_ComputeRotational(uint8_t q_key, uint8_t e_key)
{
    static Key_t  CCW_key, CW_key;           /* Q/E 键消抖状态机实例   */
    static uint8_t CCW_state = 0U, CW_state = 0U; /* Toggle 输出：0=未激活，1=已激活 */
    static uint8_t state = 0U;               /* 0=随动, 1=逆时针, 2=顺时针 */

    float yaw_angle = Gimbal_GetYawRelativeAngle();

    /* 每帧更新消抖状态机，完整按下+松开后翻转对应 state 变量 */
    Key_Scan(&CCW_key, q_key, &CCW_state);
    Key_Scan(&CW_key,  e_key, &CW_state);

    switch (state)
    {
        case 0U: /* ── 随动模式 ── */
        {
            /*
             * Q/E 均未激活：底盘随云台偏转角随动。
             * yaw_angle > 0（云台逆时针偏）→ 底盘需顺时针纠正（负速度）。
             * yaw_angle 在死区内视为已对齐，归零防止持续抖动。
             */
            if (fabsf(yaw_angle) < VTM_YAW_DEADBAND)
                s_rotational_velocity = 0.0f;
            else
                s_rotational_velocity = -yaw_angle * VTM_YAW_FOLLOW_GAIN;

            /* 点按 Q → 进入逆时针小陀螺 */
            if (CCW_state == 1U)
                state = 1U;
            /* 点按 E → 进入顺时针小陀螺（若 Q/E 同帧激活，E 优先）*/
            else if (CW_state == 1U)
                state = 2U;
            break;
        }

        case 1U: /* ── 逆时针小陀螺 ── */
        {
            s_rotational_velocity += VTM_ROTATION_ACCEL;
            if (s_rotational_velocity > VTM_MAX_ROTATIONAL_SPEED)
                s_rotational_velocity = VTM_MAX_ROTATIONAL_SPEED;

            /* 再次点按 Q（CCW_state 翻回 0）→ 退出小陀螺，回到随动 */
            if (CCW_state == 0U)
                state = 0U;

            /* 点按 E → 切换到顺时针，同时重置 CCW_state
             * 重置原因：若不清零，下帧 CCW_state==0 会误触发 state→0 */
            if (CW_state == 1U)
            {
                state = 2U;
                CCW_state = 0U; /* 消费掉 Q 的激活状态，需重新点按才能再次激活 */
            }
            break;
        }

        case 2U: /* ── 顺时针小陀螺 ── */
        {
            s_rotational_velocity -= VTM_ROTATION_ACCEL;
            if (s_rotational_velocity < -VTM_MAX_ROTATIONAL_SPEED)
                s_rotational_velocity = -VTM_MAX_ROTATIONAL_SPEED;

            /* 再次点按 E（CW_state 翻回 0）→ 退出小陀螺，回到随动 */
            if (CW_state == 0U)
                state = 0U;

            /* 点按 Q → 切换到逆时针，同时重置 CW_state */
            if (CCW_state == 1U)
            {
                state = 1U;
                CW_state = 0U; /* 消费掉 E 的激活状态，需重新点按才能再次激活 */
            }
            break;
        }

        default:
            state = 0U;
            break;
    }
}

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
            if (fabsf(yaw_angle) < VTM_YAW_DEADBAND)
                s_rotational_velocity = 0.0f;
            else
                s_rotational_velocity = -yaw_angle * VTM_YAW_FOLLOW_GAIN;
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
 * @brief  VTM 图传模块应用控制函数
 */
void VTM_Control(VTM_ctrl_t *vtm_ctrl)
{
    /* 获取本帧 Yaw 相对角（供底盘坐标变换和随动计算使用）*/
    float yaw_rel = Gimbal_GetYawRelativeAngle();

    switch (vtm_ctrl->rc.mode_switch)
    {
        /* ── mode 0：键鼠模式 ── */
        case 0U:
        {
            /* 计算平移速度（W/S/A/D 加速度控制）*/
            VTM_ComputeTranslational(vtm_ctrl);

            /* 计算旋转速度（Q=逆时针加速, E=顺时针加速, 均未按=随动）*/
            Keyboard_ComputeRotational(vtm_ctrl->keyboard.q, vtm_ctrl->keyboard.e);

            /* 底盘：平移 + 旋转指令 */
            Chassis_Control(s_longitudinal_velocity,
                            s_lateral_velocity,
                            s_rotational_velocity,
                            yaw_rel);

            /* 云台：鼠标控制 Yaw/Pitch 角速度（X 轴右移→云台右偏，取反）*/
            Gimbal_Control(-(float)vtm_ctrl->mouse.x_axis * VTM_MOUSE_GIMBAL_SENSITIVITY,
                            (float)vtm_ctrl->mouse.y_axis * VTM_MOUSE_GIMBAL_SENSITIVITY);

            /* 发射：鼠标中键=安全键, 左键=连发, 右键=单发 */
            Shooter_Control(vtm_ctrl->mouse.middle_button,
                            vtm_ctrl->mouse.left_button,
                            vtm_ctrl->mouse.right_button);
            break;
        }

        /* ── mode 2：摇杆模式 ── */
        case 2U:
        {
            /* 计算旋转速度（拨盘输入）*/
            RC_ComputeRotational(vtm_ctrl->rc.dial, yaw_rel);

            /* 底盘：摇杆通道换算为速度，缩小 SCALE 倍防止过快 */
            Chassis_Control(
                ((float)vtm_ctrl->rc.channel[2] - VTM_RC_CH_CENTER) / VTM_RC_SPEED_SCALE,
                (VTM_RC_CH_CENTER - (float)vtm_ctrl->rc.channel[3]) / VTM_RC_SPEED_SCALE,
                s_rotational_velocity,
                yaw_rel);

            /* 云台：摇杆通道控制 Yaw/Pitch */
            Gimbal_Control(
                (VTM_RC_CH_CENTER  - (float)vtm_ctrl->rc.channel[0]) * VTM_RC_GIMBAL_SENSITIVITY,
                ((float)vtm_ctrl->rc.channel[1] - VTM_RC_CH_CENTER)  * VTM_RC_GIMBAL_SENSITIVITY);

            /* 发射：暂停键=安全键, 自定义键 0=连发, 自定义键 1=单发 */
            Shooter_Control(vtm_ctrl->rc.pause_button,
                            vtm_ctrl->rc.custom_button[0],
                            vtm_ctrl->rc.custom_button[1]);
            break;
        }

        /* ── mode 1：安全停机模式 ── */
        case 1U:
        {
            /*
             * 向所有电机发送零转矩帧，强制停机。
             * 此处直接调用 BSP_CAN_TxData 而非控制层接口，
             * 是因为需要绕过控制层状态机立即停机（紧急停止语义）。
             */
            uint8_t zero[8] = {0U};

            /* 底盘 3508 清零（hcan1）*/
            BSP_CAN_TxData(CHASSIS_CAN_HANDLE, M2006_M3508_CTRL_ID_GROUP1, zero);

            /* 云台 6020 清零（hcan2）*/
            BSP_CAN_TxData(GIMBAL_CAN_HANDLE, GM6020_CTRL_ID_GROUP1, zero);

            /* 拨弹盘 M2006 清零（hcan2, 0x200）*/
            BSP_CAN_TxData(SHOOTER_CAN_HANDLE, M2006_M3508_CTRL_ID_GROUP1, zero);

            /* 摩擦轮待机 */
            FrictionWheel_SetIdle();

            /* 旋转速度归零，防止退出安全模式时底盘突然旋转 */
            s_rotational_velocity = 0.0f;
            break;
        }

        default:
            break;
    }
}
