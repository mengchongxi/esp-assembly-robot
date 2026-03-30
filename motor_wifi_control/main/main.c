/*
 * main.c – 程序入口
 * 初始化各模块，启动 HTTP 服务器和 100Hz 控制任务。
 */
#include "config.h"
#include "encoder_hw.h"
#include "motor_multi.h"
#include "joint_manager.h"
#include "control_task.h"
#include "wifi_manager.h"
#include "http_server.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

void app_main(void)
{
    /* NVS 初始化（WiFi 驱动和参数持久化依赖） */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* 硬件初始化 */
    encoder_hw_init();
    encoder_hw_load_zeros();
    motor_multi_init();

    /* 关节管理器初始化（加载 NVS 中的 PD 参数和限位） */
    joint_manager_init();

    ESP_LOGI(TAG, "硬件初始化完成");

    /* WiFi 连接（阻塞直到成功） */
    ESP_ERROR_CHECK(wifi_manager_init());

    /* HTTP 服务器 */
    ESP_ERROR_CHECK(http_server_start());

    /* 启动 100Hz 闭环控制任务 */
    control_task_start();

    ESP_LOGI(TAG, "系统启动完成，控制任务运行中");
}
