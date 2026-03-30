#pragma once

#include "esp_err.h"

/**
 * @brief 启动 HTTP 服务器并注册所有路由
 *
 * 路由列表：
 *   GET /           返回电机控制页面 HTML（含角度表盘）
 *   GET /control    方向控制  ?dir=forward|reverse|stop  &speed=0-255
 *   GET /timed      定时运行  ?dir=forward|reverse  &speed=0-255  &duration=秒
 *   GET /status     返回 JSON 状态（供前端轮询）
 *   GET /pulse      GPIO 脉冲  ?pin=A|B  返回 JSON {"action":"..."}
 *   GET /pulse-ui   返回 GPIO 脉冲控制页面 HTML
 */
esp_err_t http_server_start(void);
