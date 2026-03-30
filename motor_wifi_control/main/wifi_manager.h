#pragma once

#include "esp_err.h"

/**
 * @brief 初始化 TCP/IP 栈、事件循环，配置静态 IP 并连接 WiFi
 *        阻塞直到连接成功或重试次数耗尽
 * @return ESP_OK 连接成功，ESP_FAIL 连接失败
 */
esp_err_t wifi_manager_init(void);
