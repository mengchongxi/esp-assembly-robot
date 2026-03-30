/*
 * joint_manager.c – 关节管理器
 * 整合编码器、电机、PD 控制器，提供统一的关节控制接口。
 * 使用 mutex 保护共享状态（控制任务与 HTTP 任务并发访问）。
 * 参数通过 NVS 持久化。
 */
#include "joint_manager.h"
#include "encoder_hw.h"
#include "motor_multi.h"
#include "pd_controller.h"
#include "config.h"

#include "esp_log.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <string.h>
#include <stdio.h>

static const char *TAG = "joint";

static joint_state_t s_joints[NUM_JOINTS];
static SemaphoreHandle_t s_mutex;

/* ── NVS 辅助 ─────────────────────────────────────────────────────────── */

static void nvs_save_float(const char *key, float val)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, key, &val, sizeof(float));
        nvs_commit(h);
        nvs_close(h);
    }
}

static float nvs_load_float(const char *key, float def)
{
    nvs_handle_t h;
    float val = def;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        size_t sz = sizeof(float);
        if (nvs_get_blob(h, key, &val, &sz) != ESP_OK) val = def;
        nvs_close(h);
    }
    return val;
}

static void nvs_save_i32(const char *key, int32_t val)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, key, val);
        nvs_commit(h);
        nvs_close(h);
    }
}

static int32_t nvs_load_i32(const char *key, int32_t def)
{
    nvs_handle_t h;
    int32_t val = def;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_i32(h, key, &val) != ESP_OK) val = def;
        nvs_close(h);
    }
    return val;
}

/* ── 加载单个关节的 NVS 参数 ──────────────────────────────────────────── */
static void load_joint_params(int id)
{
    joint_state_t *j = &s_joints[id];
    char key[16];

    /* 限位 */
    snprintf(key, sizeof(key), "j%d_min", id);
    j->min_limit = nvs_load_float(key, JOINT_DEFAULT_MIN_LIMIT);
    snprintf(key, sizeof(key), "j%d_max", id);
    j->max_limit = nvs_load_float(key, JOINT_DEFAULT_MAX_LIMIT);

    /* PD 参数 */
    snprintf(key, sizeof(key), "j%d_kp", id);
    j->pd.kp = nvs_load_float(key, PD_DEFAULT_KP);
    snprintf(key, sizeof(key), "j%d_kd", id);
    j->pd.kd = nvs_load_float(key, PD_DEFAULT_KD);
    snprintf(key, sizeof(key), "j%d_ki", id);
    j->pd.ki = nvs_load_float(key, PD_DEFAULT_KI);
    snprintf(key, sizeof(key), "j%d_dz", id);
    j->pd.dead_zone = nvs_load_float(key, PD_DEFAULT_DEAD_ZONE);
    snprintf(key, sizeof(key), "j%d_mp", id);
    j->pd.min_pwm = (int)nvs_load_i32(key, PD_DEFAULT_MIN_PWM);
    snprintf(key, sizeof(key), "j%d_il", id);
    j->pd.integral_limit = nvs_load_float(key, PD_DEFAULT_INTEGRAL_LIMIT);

    ESP_LOGI(TAG, "关节%d 参数: Kp=%.2f Kd=%.2f Ki=%.2f dz=%.1f mp=%d lim=[%.0f,%.0f]",
             id, j->pd.kp, j->pd.kd, j->pd.ki,
             j->pd.dead_zone, j->pd.min_pwm, j->min_limit, j->max_limit);
}

/* ── 公共 API ─────────────────────────────────────────────────────────── */

void joint_manager_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex);

    memset(s_joints, 0, sizeof(s_joints));

    for (int i = 0; i < NUM_JOINTS; i++) {
        pd_init(&s_joints[i].pd);
        s_joints[i].enabled      = false;
        s_joints[i].target_angle = 0.0f;
        s_joints[i].min_limit    = JOINT_DEFAULT_MIN_LIMIT;
        s_joints[i].max_limit    = JOINT_DEFAULT_MAX_LIMIT;

        load_joint_params(i);
    }

    ESP_LOGI(TAG, "关节管理器初始化完成");
}

bool joint_set_target(int joint_id, float angle_deg)
{
    if (joint_id < 0 || joint_id >= NUM_JOINTS) return false;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    joint_state_t *j = &s_joints[joint_id];

    /* 限位检查 */
    if (angle_deg < j->min_limit || angle_deg > j->max_limit) {
        xSemaphoreGive(s_mutex);
        ESP_LOGW(TAG, "关节%d 目标 %.1f° 超限 [%.1f, %.1f]",
                 joint_id, angle_deg, j->min_limit, j->max_limit);
        return false;
    }

    j->target_angle = angle_deg;
    j->enabled      = true;
    pd_reset(&j->pd);   // 重置 PD 状态，避免突变

    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "关节%d 目标: %.1f°", joint_id, angle_deg);
    return true;
}

const joint_state_t* joint_get_state(int joint_id)
{
    if (joint_id < 0 || joint_id >= NUM_JOINTS) return NULL;
    return &s_joints[joint_id];
}

void joint_set_zero(int joint_id)
{
    if (joint_id < 0 || joint_id >= NUM_JOINTS) return;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    encoder_hw_set_zero(joint_id);
    s_joints[joint_id].target_angle  = 0.0f;
    s_joints[joint_id].current_angle = 0.0f;
    s_joints[joint_id].enabled       = false;
    pd_reset(&s_joints[joint_id].pd);

    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "关节%d 零点已校准", joint_id);
}

void joint_set_limits(int joint_id, float min_deg, float max_deg)
{
    if (joint_id < 0 || joint_id >= NUM_JOINTS) return;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    s_joints[joint_id].min_limit = min_deg;
    s_joints[joint_id].max_limit = max_deg;

    xSemaphoreGive(s_mutex);

    char key[16];
    snprintf(key, sizeof(key), "j%d_min", joint_id);
    nvs_save_float(key, min_deg);
    snprintf(key, sizeof(key), "j%d_max", joint_id);
    nvs_save_float(key, max_deg);

    ESP_LOGI(TAG, "关节%d 限位: [%.1f, %.1f]°", joint_id, min_deg, max_deg);
}

void joint_set_pd(int joint_id, float kp, float kd, float ki,
                  float dead_zone, int min_pwm)
{
    if (joint_id < 0 || joint_id >= NUM_JOINTS) return;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    pd_controller_t *pd = &s_joints[joint_id].pd;
    pd->kp        = kp;
    pd->kd        = kd;
    pd->ki        = ki;
    pd->dead_zone = dead_zone;
    pd->min_pwm   = min_pwm;

    xSemaphoreGive(s_mutex);

    /* 存入 NVS */
    char key[16];
    snprintf(key, sizeof(key), "j%d_kp", joint_id);
    nvs_save_float(key, kp);
    snprintf(key, sizeof(key), "j%d_kd", joint_id);
    nvs_save_float(key, kd);
    snprintf(key, sizeof(key), "j%d_ki", joint_id);
    nvs_save_float(key, ki);
    snprintf(key, sizeof(key), "j%d_dz", joint_id);
    nvs_save_float(key, dead_zone);
    snprintf(key, sizeof(key), "j%d_mp", joint_id);
    nvs_save_i32(key, (int32_t)min_pwm);

    ESP_LOGI(TAG, "关节%d PD: Kp=%.2f Kd=%.2f Ki=%.2f dz=%.1f mp=%d",
             joint_id, kp, kd, ki, dead_zone, min_pwm);
}

void joint_emergency_stop(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    for (int i = 0; i < NUM_JOINTS; i++) {
        s_joints[i].enabled = false;
        pd_reset(&s_joints[i].pd);
    }

    xSemaphoreGive(s_mutex);

    motor_multi_stop_all();
    ESP_LOGW(TAG, "急停！所有关节已禁用");
}

void joint_update_all(float dt)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    for (int i = 0; i < NUM_JOINTS; i++) {
        joint_state_t *j = &s_joints[i];

        /* 读取当前角度 */
        j->current_angle = encoder_hw_get_angle(i);

        if (!j->enabled) continue;

        /* PD 计算 */
        int pwm = pd_compute(&j->pd, j->target_angle, j->current_angle, dt);

        /* 输出到电机 */
        motor_multi_set_speed(i, pwm);
    }

    xSemaphoreGive(s_mutex);
}
