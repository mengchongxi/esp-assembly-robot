/*
 * control_task.c – FreeRTOS 闭环控制任务
 * 以 100Hz 频率运行 PD 位置控制循环：
 *   读取编码器 → PD 计算 → 设置电机 PWM
 */
#include "control_task.h"
#include "joint_manager.h"
#include "config.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ctrl";
static TaskHandle_t s_task_handle = NULL;
static volatile bool s_running = false;

static void control_loop(void *arg)
{
    const float dt = (float)CONTROL_LOOP_PERIOD_MS / 1000.0f;
    const TickType_t period = pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS);

    ESP_LOGI(TAG, "控制任务启动: %dHz, dt=%.3fs", 1000 / CONTROL_LOOP_PERIOD_MS, dt);

    TickType_t last_wake = xTaskGetTickCount();

    while (s_running) {
        joint_update_all(dt);
        vTaskDelayUntil(&last_wake, period);
    }

    ESP_LOGI(TAG, "控制任务已停止");
    s_task_handle = NULL;
    vTaskDelete(NULL);
}

void control_task_start(void)
{
    if (s_task_handle != NULL) {
        ESP_LOGW(TAG, "控制任务已在运行");
        return;
    }

    s_running = true;
    BaseType_t ret = xTaskCreatePinnedToCore(
        control_loop,
        "ctrl_loop",
        CONTROL_TASK_STACK_SIZE,
        NULL,
        CONTROL_TASK_PRIORITY,
        &s_task_handle,
        1       // 固定在核心 1（核心 0 留给 WiFi/系统）
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "控制任务创建失败");
        s_running = false;
    }
}

void control_task_stop(void)
{
    s_running = false;
    ESP_LOGI(TAG, "请求停止控制任务...");
}
