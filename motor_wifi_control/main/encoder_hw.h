#pragma once

#include "config.h"

/**
 * @brief 初始化所有编码器的 MCPWM 捕获通道
 *        每个编码器使用 1 个捕获通道，捕获上升沿+下降沿
 */
void encoder_hw_init(void);

/**
 * @brief 获取指定关节的当前角度（多圈，含零点偏移）
 * @param joint_id 关节索引 0-4
 * @return 角度（度），可超出 0-360 范围（多圈）
 */
float encoder_hw_get_angle(int joint_id);

/**
 * @brief 获取原始角度（0-360°，无偏移）
 * @param joint_id 关节索引 0-4
 * @return 0.0-360.0 度，或 -1.0 表示无有效数据
 */
float encoder_hw_get_raw_angle(int joint_id);

/**
 * @brief 设定当前位置为零点（存入 NVS）
 * @param joint_id 关节索引 0-4
 */
void encoder_hw_set_zero(int joint_id);

/**
 * @brief 从 NVS 加载所有关节的零点偏移
 */
void encoder_hw_load_zeros(void);
