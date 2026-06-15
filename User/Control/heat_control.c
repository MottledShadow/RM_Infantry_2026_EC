/**
 ******************************************************************************
 * @file    heat_control.c
 * @brief   发射热量控制实现（控制层）
 * @note    职责边界：
 *            - GPIO 读取：委托给 bsp_gpio（BSP 层）
 *            - 光电门消抖：复用 Key_Scan（算法层）
 *            - 热量积分 / 衰减 / 目标转速计算：本层实现
 *          【禁止】在本文件中直接调用 HAL_GPIO_* 或访问 GPIOx 寄存器
 * @version 1.0
 * @date    2026-4-6
 * @author  黄仲华
 ******************************************************************************
 */

#include "heat_control.h"

/* ========================================================================== */
/*                              私有函数声明                                    */
/* ========================================================================== */

static void  HC_DetectShot(HeatControl_t *hc);
static void  HC_CalcTargetOmega(HeatControl_t *hc);
static float HC_ShotsToOmega(const HeatControl_t *hc, float shots_per_sec);

/* ========================================================================== */
/*                              公开函数实现                                    */
/* ========================================================================== */

/**
 * @brief  热量控制初始化
 */
void HeatControl_Init(HeatControl_t *hc, const HeatControl_Config_t *cfg)
{
    /* ── 加载静态赛规参数（优先使用传入配置，否则使用默认值）── */
    if (cfg != NULL)
    {
        hc->heat_per_shot = cfg->heat_per_shot;
        hc->q_warn        = cfg->q_warn;
        hc->q_full        = cfg->q_full;
        hc->q_stop        = cfg->q_stop;
        hc->n_max         = cfg->n_max;
    }
    else
    {
        hc->heat_per_shot = HC_DEFAULT_HEAT_PER_SHOT;
        hc->q_warn        = HC_DEFAULT_Q_WARN;
        hc->q_full        = HC_DEFAULT_Q_FULL;
        hc->q_stop        = HC_DEFAULT_Q_STOP;
        hc->n_max         = HC_DEFAULT_N_MAX;
    }

    /*
     * ── 初始化动态参数（来自裁判系统，每帧由 Update 同步）──
     * 此处读取当前裁判系统值作为初始值。
     * 调用方须保证：在调用本函数之前，已等待裁判系统数据就绪
     * （即 g_rsi_robot_status.robot_id != 0），否则以下值为 0，
     * Update 第一帧会立即用正确值覆盖，影响最多一帧，可接受。
     */
    hc->q_max      = (float)g_rsi_robot_status.shooter_barrel_heat_limit;
    hc->q_cooldown = (float)g_rsi_robot_status.shooter_barrel_cooling_value;

    /* 预计算固定量 */
    hc->shots_per_2006_rev = HC_SHOTS_PER_DISC_REV / HC_MOTOR2006_REDUCTION_RATIO;

    /* 清零运行时状态 */
    hc->q_now                = 0.0f;
    hc->current_target_omega = 0.0f;
    hc->shot_detected        = 0U;

    /* 初始化光电门消抖状态机 */
    hc->shot_key.state = KEY_STATE_IDLE;
    hc->shot_key.cnt   = 0U;
}

/**
 * @brief  热量控制每帧更新
 */
void HeatControl_Update(HeatControl_t *hc)
{
    /*
     * Step 0：同步裁判系统动态参数
     * 热量上限和冷却速率随机器人等级/比赛进程实时变化，
     * 必须每帧从 g_rsi_robot_status 读取最新值，
     * 不能在 Init 时固定（这是原版设计的根本性错误）。
     *
     * Fallback 保护：裁判系统未连接时字段为 0，
     * q_max==0 会导致 Qres 始终为负，发射完全禁止，
     * 调试时使用保守默认值代替，正式比赛不会触发此分支。
     */
    {
        float q_max_raw = (float)g_rsi_robot_status.shooter_barrel_heat_limit;
        float q_cd_raw  = (float)g_rsi_robot_status.shooter_barrel_cooling_value;
        hc->q_max      = (g_rsi_robot_status.robot_id > 0.0f) ? q_max_raw : HC_FALLBACK_Q_MAX;
        hc->q_cooldown = (g_rsi_robot_status.robot_id > 0.0f) ? q_cd_raw  : HC_FALLBACK_Q_COOLDOWN;
    }

    /* Step 1：光电门消抖检测，判断本帧是否有完整弹丸通过事件 */
    HC_DetectShot(hc);

    /* Step 2：检测到弹丸通过则累加热量 */
    if (hc->shot_detected)
    {
        hc->q_now        += hc->heat_per_shot;
        hc->shot_detected  = 0U;
    }

    /* Step 3：热量冷却衰减（每帧衰减量 = 冷却速率 / 更新频率）*/
    hc->q_now -= hc->q_cooldown / (float)HC_UPDATE_FREQ_HZ;
    if (hc->q_now < 0.0f)
        hc->q_now = 0.0f;

    /* Step 4：根据当前剩余热量更新目标转速 */
    HC_CalcTargetOmega(hc);
}

/**
 * @brief  获取当前目标拨弹盘转速
 */
float HeatControl_GetTargetOmega(const HeatControl_t *hc)
{
    return hc->current_target_omega;
}

/**
 * @brief  手动预扣单发热量
 */
void HeatControl_AddHeat(HeatControl_t *hc)
{
    hc->q_now += hc->heat_per_shot;
}

/* ========================================================================== */
/*                              私有函数实现                                    */
/* ========================================================================== */

/**
 * @brief  读取光电门 GPIO 并通过 Key_Scan 消抖，判断是否有完整弹丸通过事件
 * @note   Key_Scan 为 Toggle 型：完整按下+松开后翻转 shot_detected。
 *         此处 shot_detected 扮演"Toggle 输出"的角色：
 *         每检测到一次完整通过事件，shot_detected 由 0 翻转为 1；
 *         Update 函数读取后立即清零，等待下一次触发。
 *
 *         ⚠️  注意：Key_Scan 的 Toggle 语义是"翻转"，不是"置 1"。
 *             若 shot_detected 初始为 0，第一次触发后变为 1（正确）；
 *             若因某种原因未被 Update 及时清零（如调用频率异常），
 *             第二次触发会将其翻转回 0，导致漏计。
 *             因此务必保证 Update 在每次 HC_DetectShot 之后被调用。
 */
static void HC_DetectShot(HeatControl_t *hc)
{
    /* 从 BSP 层读取当前 GPIO 电平（1=遮光/有弹丸，0=通畅/无弹丸）*/
    uint8_t pin_level = BSP_GPIO_ReadShotDetect();

    /* 调用算法层 Key_Scan 消抖，检测到完整通过事件时翻转 shot_detected */
    Key_Scan(&hc->shot_key, pin_level, &hc->shot_detected);
}

/**
 * @brief  将发射频率（发/s）换算为 2006 电机转速（RPM）
 * @param  hc              热量控制实例
 * @param  shots_per_sec   发射频率（发/s）
 * @retval 对应的 2006 电机转速（RPM）
 */
static float HC_ShotsToOmega(const HeatControl_t *hc, float shots_per_sec)
{
    /* shots_per_2006_rev: 2006 每转发弹数
     * shots_per_sec / shots_per_2006_rev = 2006 转速（转/s）
     * × 60 → RPM                                               */
    return (shots_per_sec / hc->shots_per_2006_rev) * 60.0f;
}

/**
 * @brief  根据剩余热量计算目标拨弹盘转速
 * @note   四段策略（基于 Qres = Qmax - Qnow）：
 *
 *         Qres ≥ q_warn              → n = n_max（全速）
 *         q_full ≤ Qres < q_warn     → n 线性插值 [0, n_max]
 *         q_stop ≤ Qres < q_full     → n = Qcd / heat_per_shot（冷却维持速）
 *         Qres < q_stop              → n = 0（禁发）
 *
 *         ⚠️  当 q_full == q_stop 时（当前默认配置均为 15.0f），
 *             中间段 [q_stop, q_full) 为空区间，策略退化为三段。
 *             这是预期行为，如需四段策略请将 q_full 设置大于 q_stop。
 */
static void HC_CalcTargetOmega(HeatControl_t *hc)
{
    float q_res = hc->q_max - hc->q_now;  /* 剩余热量余量 */
    float n;                               /* 目标发射频率（发/s）*/

    if (q_res >= hc->q_warn)
    {
        /* 余量充足，全速发射 */
        n = hc->n_max;
    }
    else if (q_res >= hc->q_full)
    {
        /* 余量警戒区，线性降速：ratio ∈ [0, 1) */
        float ratio = (q_res - hc->q_full) / (hc->q_warn - hc->q_full);
        n = hc->n_max * ratio;
    }
    else if (q_res >= hc->q_stop)
    {
        /* 余量极低，以冷却速率维持最低发射频率（勉强不超热）*/
        n = hc->q_cooldown / hc->heat_per_shot;
    }
    else
    {
        /* 余量耗尽，强制停发 */
        n = 0.0f;
    }

    hc->current_target_omega = HC_ShotsToOmega(hc, n);
}
