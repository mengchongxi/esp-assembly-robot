#pragma once

#include "config.h"
#include "pd_controller.h"

#include <stdbool.h>

/**
 * 关节状态（由 joint_manager 内部维护，外部只读）
 */
typedef struct {
    float            target_angle;   // 目标角度（度）
    float            current_angle;  // 当前角度（度，含多圈+零点偏移）
    float            min_limit;      // 最小限位（度）
    float            max_limit;      // 最大限位（度）
    bool             enabled;        // 是否启用 PD 控制
    pd_controller_t  pd;             // PD 控制器
} joint_state_t;

/**
 * @brief 初始化关节管理器（加载 NVS 参数）
 */
void joint_manager_init(void);

/**
 * @brief 设定关节目标角度（自动限位检查）
 * @return true 成功，false 超限被拒绝
 */
bool joint_set_target(int joint_id, float angle_deg);

/**
 * @brief 获取关节状态（线程安全）
 */
const joint_state_t* joint_get_state(int joint_id);

/**
 * @brief 校准关节零点（存 NVS）
 */
void joint_set_zero(int joint_id);

/**
 * @brief 设定关节限位（存 NVS）
 */
void joint_set_limits(int joint_id, float min_deg, float max_deg);

/**
 * @brief 设定关节 PD 参数（存 NVS）
 */
void joint_set_pd(int joint_id, float kp, float kd, float ki,
                  float dead_zone, int min_pwm);

/**
 * @brief 急停所有关节：停止电机并禁用控制
 */
void joint_emergency_stop(void);

/**
 * @brief 控制循环更新（由 control_task 以固定频率调用）
 * @param dt 控制周期（秒）
 */
void joint_update_all(float dt);
