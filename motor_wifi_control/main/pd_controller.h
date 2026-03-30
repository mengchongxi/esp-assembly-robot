#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * PD/PID 控制器（Ki 预留，初始为纯 PD）
 */
typedef struct {
    float kp;               // 比例增益
    float kd;               // 微分增益
    float ki;               // 积分增益（预留，初始 0 = 纯 PD）
    float dead_zone;        // 死区角度（度）
    int   min_pwm;          // 最小启动 PWM（摩擦补偿）
    float prev_error;       // 上一次误差（内部状态）
    float integral_error;   // 积分误差累积（预留）
    float integral_limit;   // 积分限幅（防饱和）
} pd_controller_t;

/**
 * @brief 初始化 PD 控制器为默认参数
 */
void pd_init(pd_controller_t *pd);

/**
 * @brief 计算 PD 控制输出
 * @param pd      控制器实例
 * @param target  目标角度（度）
 * @param current 当前角度（度）
 * @param dt      控制周期（秒）
 * @return -255 ~ 255 的 PWM 值
 */
int pd_compute(pd_controller_t *pd, float target, float current, float dt);

/**
 * @brief 重置控制器内部状态（误差累积等）
 */
void pd_reset(pd_controller_t *pd);
