#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_http_server.h"

/**
 * @brief 发送电机控制 HTML 页面（含绝对角度读数、定时运行）
 */
esp_err_t web_ui_send_html(httpd_req_t *req);

#ifdef __cplusplus
}
#endif
