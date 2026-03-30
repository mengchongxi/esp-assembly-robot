/*
 * http_server.c – HTTP 服务器与 REST API
 * 提供 5 关节控制 API 和 Web UI 页面。
 */
#include "http_server.h"
#include "web_ui.h"
#include "joint_manager.h"
#include "config.h"

#include "esp_http_server.h"
#include "esp_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "http";

/* ── 查询参数辅助 ─────────────────────────────────────────────────────── */

static void get_query_param(httpd_req_t *req, const char *key,
                             char *out, size_t out_len)
{
    size_t qlen = httpd_req_get_url_query_len(req);
    if (qlen == 0) { out[0] = '\0'; return; }

    char *buf = malloc(qlen + 1);
    if (!buf) { out[0] = '\0'; return; }

    httpd_req_get_url_query_str(req, buf, qlen + 1);
    if (httpd_query_key_value(buf, key, out, out_len) != ESP_OK) {
        out[0] = '\0';
    }
    free(buf);
}

/* ── JSON 状态生成 ────────────────────────────────────────────────────── */

static void send_json_status(httpd_req_t *req)
{
    /* 预分配足够大的缓冲区：每个关节约 200 字节 */
    char json[1280];
    int pos = 0;

    pos += snprintf(json + pos, sizeof(json) - pos, "{\"joints\":[");

    for (int i = 0; i < NUM_JOINTS; i++) {
        const joint_state_t *j = joint_get_state(i);
        if (!j) continue;

        if (i > 0) pos += snprintf(json + pos, sizeof(json) - pos, ",");

        pos += snprintf(json + pos, sizeof(json) - pos,
            "{\"id\":%d,"
            "\"current\":%.2f,"
            "\"target\":%.2f,"
            "\"enabled\":%s,"
            "\"limits\":[%.1f,%.1f],"
            "\"pd\":{\"kp\":%.2f,\"kd\":%.2f,\"ki\":%.2f,"
            "\"dead\":%.1f,\"min_pwm\":%d}}",
            i,
            j->current_angle,
            j->target_angle,
            j->enabled ? "true" : "false",
            j->min_limit, j->max_limit,
            j->pd.kp, j->pd.kd, j->pd.ki,
            j->pd.dead_zone, j->pd.min_pwm);
    }

    pos += snprintf(json + pos, sizeof(json) - pos, "]}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, pos);
}

static void send_json_ok(httpd_req_t *req, const char *msg)
{
    char json[128];
    int len = snprintf(json, sizeof(json), "{\"ok\":true,\"msg\":\"%s\"}", msg);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, len);
}

static void send_json_err(httpd_req_t *req, const char *msg)
{
    char json[128];
    int len = snprintf(json, sizeof(json), "{\"ok\":false,\"msg\":\"%s\"}", msg);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, len);
}

/* ── 路由处理 ─────────────────────────────────────────────────────────── */

/* GET / — Web UI 页面 */
static esp_err_t handle_root(httpd_req_t *req)
{
    return web_ui_send_html(req);
}

/* GET /status — 返回所有关节状态 */
static esp_err_t handle_status(httpd_req_t *req)
{
    send_json_status(req);
    return ESP_OK;
}

/* GET /joint?id=0-4&target=角度 — 设定目标角度 */
static esp_err_t handle_joint(httpd_req_t *req)
{
    char id_str[8] = {0}, target_str[16] = {0};
    get_query_param(req, "id",     id_str,     sizeof(id_str));
    get_query_param(req, "target", target_str, sizeof(target_str));

    int id = atoi(id_str);
    float target = atof(target_str);

    if (joint_set_target(id, target)) {
        send_json_ok(req, "target set");
    } else {
        send_json_err(req, "out of limits");
    }
    return ESP_OK;
}

/* GET /zero?id=0-4 — 校准零点 */
static esp_err_t handle_zero(httpd_req_t *req)
{
    char id_str[8] = {0};
    get_query_param(req, "id", id_str, sizeof(id_str));

    int id = atoi(id_str);
    if (id < 0 || id >= NUM_JOINTS) {
        send_json_err(req, "invalid id");
        return ESP_OK;
    }

    joint_set_zero(id);
    send_json_ok(req, "zero set");
    return ESP_OK;
}

/* GET /limits?id=0-4&min=度&max=度 — 设定限位 */
static esp_err_t handle_limits(httpd_req_t *req)
{
    char id_str[8] = {0}, min_str[16] = {0}, max_str[16] = {0};
    get_query_param(req, "id",  id_str,  sizeof(id_str));
    get_query_param(req, "min", min_str, sizeof(min_str));
    get_query_param(req, "max", max_str, sizeof(max_str));

    int id = atoi(id_str);
    float min_v = atof(min_str);
    float max_v = atof(max_str);

    if (id < 0 || id >= NUM_JOINTS) {
        send_json_err(req, "invalid id");
        return ESP_OK;
    }

    joint_set_limits(id, min_v, max_v);
    send_json_ok(req, "limits set");
    return ESP_OK;
}

/* GET /pd?id=0-4&kp=&kd=&ki=&dead=&min_pwm= — 设定 PD 参数 */
static esp_err_t handle_pd(httpd_req_t *req)
{
    char id_str[8] = {0}, kp_s[16] = {0}, kd_s[16] = {0};
    char ki_s[16] = {0}, dz_s[16] = {0}, mp_s[8] = {0};

    get_query_param(req, "id",       id_str, sizeof(id_str));
    get_query_param(req, "kp",       kp_s,   sizeof(kp_s));
    get_query_param(req, "kd",       kd_s,   sizeof(kd_s));
    get_query_param(req, "ki",       ki_s,   sizeof(ki_s));
    get_query_param(req, "dead",     dz_s,   sizeof(dz_s));
    get_query_param(req, "min_pwm",  mp_s,   sizeof(mp_s));

    int id = atoi(id_str);
    if (id < 0 || id >= NUM_JOINTS) {
        send_json_err(req, "invalid id");
        return ESP_OK;
    }

    /* 读取当前值作为未提供参数的默认值 */
    const joint_state_t *j = joint_get_state(id);
    float kp   = kp_s[0] ? atof(kp_s) : j->pd.kp;
    float kd   = kd_s[0] ? atof(kd_s) : j->pd.kd;
    float ki   = ki_s[0] ? atof(ki_s) : j->pd.ki;
    float dz   = dz_s[0] ? atof(dz_s) : j->pd.dead_zone;
    int   mp   = mp_s[0] ? atoi(mp_s) : j->pd.min_pwm;

    joint_set_pd(id, kp, kd, ki, dz, mp);
    send_json_ok(req, "pd updated");
    return ESP_OK;
}

/* GET /stop — 急停所有关节 */
static esp_err_t handle_stop(httpd_req_t *req)
{
    joint_emergency_stop();
    send_json_ok(req, "emergency stop");
    return ESP_OK;
}

/* ── 服务器启动 ───────────────────────────────────────────────────────── */

esp_err_t http_server_start(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port      = HTTP_SERVER_PORT;
    config.max_uri_handlers = 16;

    ESP_ERROR_CHECK(httpd_start(&server, &config));

    static const httpd_uri_t routes[] = {
        { .uri = "/",        .method = HTTP_GET, .handler = handle_root    },
        { .uri = "/status",  .method = HTTP_GET, .handler = handle_status  },
        { .uri = "/joint",   .method = HTTP_GET, .handler = handle_joint   },
        { .uri = "/zero",    .method = HTTP_GET, .handler = handle_zero    },
        { .uri = "/limits",  .method = HTTP_GET, .handler = handle_limits  },
        { .uri = "/pd",      .method = HTTP_GET, .handler = handle_pd      },
        { .uri = "/stop",    .method = HTTP_GET, .handler = handle_stop    },
    };

    for (int i = 0; i < (int)(sizeof(routes) / sizeof(routes[0])); i++) {
        httpd_register_uri_handler(server, &routes[i]);
    }

    ESP_LOGI(TAG, "HTTP 服务器已启动，端口 %d，%d 个路由",
             HTTP_SERVER_PORT, (int)(sizeof(routes) / sizeof(routes[0])));
    return ESP_OK;
}
