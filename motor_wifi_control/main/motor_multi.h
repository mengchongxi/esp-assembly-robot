#pragma once

#include "config.h"

/**
 * @brief 初始化所有电机的 LEDC PWM 通道
 *        配置 3 个 LEDC 定时器 + 10 个通道
 */
void motor_multi_init(void);

/**
 * @brief 设置电机速度
 * @param joint_id 关节索引 0-4
 * @param speed    -255（全速反转）~ 0（停止）~ 255（全速正转）
 */
void motor_multi_set_speed(int joint_id, int speed);

/**
 * @brief 停止指定电机
 * @param joint_id 关节索引 0-4
 */
void motor_multi_stop(int joint_id);

/**
 * @brief 急停所有电机
 */
void motor_multi_stop_all(void);
