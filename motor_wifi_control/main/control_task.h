#pragma once

/**
 * @brief 启动 100Hz 控制循环 FreeRTOS 任务
 */
void control_task_start(void);

/**
 * @brief 请求停止控制任务
 */
void control_task_stop(void);
