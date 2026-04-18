/**
 ******************************************************************************
 * @file    heat_control.h
 * @brief   发射热量控制头文件（控制层）
 * @note    层级说明：
 *            本文件属于【控制层】，位于 BSP 层之上、应用层之下。
 *            职责：根据当前热量余量动态限制拨弹盘转速，
 *                  防止触发裁判系统热量超限罚扣血量。
 *
 *          设计说明：
 *            ① GPIO 读取委托给 bsp_gpio（BSP 层）
 *            ② 弹丸光电门消抖复用 Key_Scan（算法层），消除重复代码
 *            ③ 热量计算参数通过 HeatControl_Config_t 外部传入，
 *               不在 Init 中硬编码，便于赛前按裁判系统参数调整
 *
 *          热量控制策略（基于剩余热量 Qres = Qmax - Qnow）：
 *            Qres ≥ q_warn          → 满转速发射
 *            q_full ≤ Qres < q_warn → 线性降速（ratio 插值）
 *            q_stop ≤ Qres < q_full → 以冷却速率勉强维持最低转速
 *            Qres < q_stop          → 禁止发射（转速 = 0）
 *
 *          ⚠️  注意：q_full 与 q_stop 设为相同值时，中间段为空区间，
 *              实际退化为三段策略（全速 / 最低速 / 停发），属于预期行为。
 *
 * @version 1.0
 * @date    2026-4-6
 * @author  黄仲华
 ******************************************************************************
 */

#ifndef HEAT_CONTROL_H
#define HEAT_CONTROL_H

#include <stdint.h>
#include "key_scan.h"  /* 复用 Key_t 消抖状态机 */
#include "bsp_gpio.h"   /* BSP 层：读取弹丸检测 GPIO（控制层 → BSP 层，方向正确）*/
#include "key_scan.h"   /* 算法层：复用消抖状态机（控制层 → 算法层，方向正确）*/
#include "rsi_driver.h" /* 设备驱动层：读取裁判系统动态参数（热量上限/冷却速率）*/

/* ========================================================================== */
/*                          机械参数（硬件相关，较稳定）                         */
/* ========================================================================== */

/** 拨弹盘每转发出的弹丸数（取决于拨弹盘齿数，通常为 8）*/
#define HC_SHOTS_PER_DISC_REV           8.0f

/** 2006 电机减速比（电机转速 / 拨弹盘转速）*/
#define HC_MOTOR2006_REDUCTION_RATIO    36.0f

/** 弹丸通过光电门的消抖计数（单位：调用周期数，@ 1kHz = 10ms）*/
#define HC_SHOT_DEBOUNCE_CNT            10U

/** 热量控制更新频率（Hz），用于热量冷却的每帧衰减量换算 */
#define HC_UPDATE_FREQ_HZ               1000U

/* ========================================================================== */
/*                          赛规参数（仅静态参数，动态参数由裁判系统实时下发）   */
/* ========================================================================== */

/**
 * @note  以下两个参数（热量上限和冷却速率）来自裁判系统实时数据，
 *        随机器人等级变化，HeatControl_Update() 每帧自动从
 *        g_rsi_robot_status 同步，不再使用固定默认值。
 *        HC_DEFAULT_Q_MAX / HC_DEFAULT_Q_COOLDOWN 已从代码中移除。
 *
 *        若裁判系统在 App_Init() 超时前未就绪（调试模式），
 *        HeatControl_Init 从 g_rsi_robot_status 读到的值为 0，
 *        Update() 第一帧会读到正确值覆盖（正式比赛）。
 *        调试模式下为防止热量上限为 0 导致发射完全禁止，
 *        HeatControl_Update 对 q_max==0 做了 Fallback 保护：
 *        自动使用 HC_FALLBACK_Q_MAX 和 HC_FALLBACK_Q_COOLDOWN。
 */

/**
 * @brief  裁判系统未就绪时的热量上限 Fallback 值（J）
 * @note   仅在调试模式（裁判系统未连接）下生效，取 1 级步兵默认值。
 *         正式比赛时此值永远不会被使用（裁判系统就绪后每帧覆盖）。
 */
#define HC_FALLBACK_Q_MAX           50.0f

/**
 * @brief  裁判系统未就绪时的冷却速率 Fallback 值（J/s）
 * @note   同上，取保守值防止调试时意外过热。
 */
#define HC_FALLBACK_Q_COOLDOWN      10.0f

/** 单发热量消耗（J）：17mm 弹丸约 10J，按实际标定 */
#define HC_DEFAULT_HEAT_PER_SHOT    10.0f

/** 警戒剩余热量（J）：低于此值开始线性降速 */
#define HC_DEFAULT_Q_WARN           30.0f

/** 最低限速剩余热量（J）：低于此值切换为最低维持转速 */
#define HC_DEFAULT_Q_FULL           15.0f

/** 禁发剩余热量（J）：低于此值目标转速清零 */
#define HC_DEFAULT_Q_STOP           15.0f

/** 最大发射频率（发/s）*/
#define HC_DEFAULT_N_MAX            15.0f

/* ========================================================================== */
/*                              数据结构定义                                    */
/* ========================================================================== */

/**
 * @brief  热量控制赛规参数配置（由应用层传入，便于赛前按裁判系统参数调整）
 */
typedef struct
{
    float q_max;          /**< 热量上限（J）                        */
    float q_cooldown;     /**< 每秒冷却量（J/s）                    */
    float heat_per_shot;  /**< 单发热量消耗（J）                    */
    float q_warn;         /**< 警戒剩余热量，低于此值开始降速（J）  */
    float q_full;         /**< 最低限速剩余热量（J）                */
    float q_stop;         /**< 禁止发射剩余热量（J）                */
    float n_max;          /**< 最大发射频率（发/s）                 */
} HeatControl_Config_t;

/**
 * @brief  热量控制运行时状态（由 HeatControl_Init 初始化，禁止外部直接写入）
 * @note   参数分为两类：
 *
 *         【静态参数】：比赛期间不变，由 Init 时从 Config 写入：
 *           heat_per_shot, q_warn, q_full, q_stop, n_max, shots_per_2006_rev
 *
 *         【动态参数】：由裁判系统实时下发，随机器人等级/比赛进程变化，
 *           Update() 每帧自动从 g_rsi_robot_status 同步，Init 仅做初始化：
 *           q_max    ← g_rsi_robot_status.shooter_barrel_heat_limit
 *           q_cooldown ← g_rsi_robot_status.shooter_barrel_cooling_value
 */
typedef struct
{
    /* ── 动态参数（每帧由 Update 从裁判系统同步）── */
    float q_max;        /**< 热量上限（J）= shooter_barrel_heat_limit，随等级变化      */
    float q_cooldown;   /**< 每秒冷却量（J/s）= shooter_barrel_cooling_value，随等级变化*/

    /* ── 静态参数（Init 后不变，取决于硬件和赛规固定项）── */
    float heat_per_shot;        /**< 单发热量消耗（J），17mm 约 10J                    */
    float q_warn;               /**< 警戒剩余热量，低于此值开始降速（J）              */
    float q_full;               /**< 最低限速剩余热量（J）                             */
    float q_stop;               /**< 禁止发射剩余热量（J）                             */
    float n_max;                /**< 最大发射频率（发/s）                              */

    /* ── 运行时热量状态 ── */
    float q_now;                /**< 当前累计热量估计值（J），由 Update 实时维护       */

    /* ── 预计算量（Init 时固定）── */
    float shots_per_2006_rev;   /**< 2006 每转对应发射数（= 齿数/减速比）             */

    /* ── 输出 ── */
    float current_target_omega; /**< 当前目标拨弹盘转速（RPM），控制层读取            */

    /* ── 弹丸消抖（复用 Key_Scan 算法层状态机）── */
    Key_t   shot_key;           /**< 光电门消抖状态机实例                             */
    uint8_t shot_detected;      /**< 本帧是否检测到完整弹丸通过事件                   */
} HeatControl_t;

/* ========================================================================== */
/*                              函数声明                                        */
/* ========================================================================== */

/**
 * @brief  热量控制初始化
 * @note   若 cfg 为 NULL，使用 HC_DEFAULT_* 宏中的默认赛规参数。
 *         赛前若裁判系统参数有变，构造 HeatControl_Config_t 传入即可，
 *         无需修改本文件。
 * @param  hc   热量控制实例指针
 * @param  cfg  赛规参数配置指针，NULL 使用默认值
 * @retval 无
 */
void HeatControl_Init(HeatControl_t *hc, const HeatControl_Config_t *cfg);

/**
 * @brief  热量控制每帧更新
 * @note   须以 HC_UPDATE_FREQ_HZ 的固定频率调用（通常在 1ms 定时器中）：
 *           0. 从 g_rsi_robot_status 同步动态参数（热量上限、冷却速率）
 *           1. 读取光电门 GPIO，经消抖判断是否有弹丸通过
 *           2. 有弹丸通过则累加热量
 *           3. 按冷却速率衰减热量
 *           4. 根据剩余热量更新目标转速
 * @param  hc  热量控制实例指针
 * @retval 无
 */
void HeatControl_Update(HeatControl_t *hc);

/**
 * @brief  获取当前目标拨弹盘转速
 * @param  hc  热量控制实例指针（只读）
 * @retval 目标转速（RPM）
 */
float HeatControl_GetTargetOmega(const HeatControl_t *hc);

/**
 * @brief  手动预扣单发热量
 * @note   用于单发模式下光电门响应滞后时的预防性热量扣除，
 *         防止在裁判系统计算之前发出下一发导致超热。
 * @param  hc  热量控制实例指针
 * @retval 无
 */
void HeatControl_AddHeat(HeatControl_t *hc);

#endif /* HEAT_CONTROL_H */
