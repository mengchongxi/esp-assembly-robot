/*
 * encoder_hw.c – MCPWM Capture 硬件编码器
 * 使用 ESP32 MCPWM 捕获模块读取 5 个 AS5048A 绝对编码器的 PWM 信号。
 * ISR 中捕获上升/下降沿的硬件时间戳，零 CPU 占用。
 * 支持多圈追踪和零点校准（NVS 持久化）。
 */
#include "encoder_hw.h"
#include "config.h"

#include "driver/mcpwm_cap.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <math.h>
#include <string.h>

static const char *TAG = "enc_hw";

/* ── 每个编码器通道的 ISR 状态 ─────────────────────────────────────────── */
typedef struct {
    volatile uint32_t rise_tick;        // 上升沿时间戳
    volatile uint32_t high_ticks;       // 高电平持续 tick 数
    volatile uint32_t period_ticks;     // 完整周期 tick 数
    volatile bool     data_ready;       // 标记一次完整测量已完成
} capture_isr_data_t;

/* ── 每个编码器的状态 ──────────────────────────────────────────────────── */
typedef struct {
    capture_isr_data_t  isr;            // ISR 写入的捕获数据
    float               raw_angle;      // 当前原始角度 0-360°
    float               prev_raw;       // 上一次原始角度（用于多圈检测）
    int32_t             turn_count;     // 多圈计数
    float               zero_offset;    // 零点偏移（NVS 持久化）
    mcpwm_cap_channel_handle_t cap_ch;  // 捕获通道句柄
} encoder_state_t;

static encoder_state_t s_enc[NUM_JOINTS];
static mcpwm_cap_timer_handle_t s_cap_timers[2];  // 2 个 MCPWM group
static uint32_t s_timer_resolution[2];             // 捕获定时器分辨率

/* ── ISR 回调 ─────────────────────────────────────────────────────────── */
static bool IRAM_ATTR capture_callback(mcpwm_cap_channel_handle_t cap_chan,
                                        const mcpwm_capture_event_data_t *edata,
                                        void *user_data)
{
    capture_isr_data_t *d = (capture_isr_data_t *)user_data;

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        /* 上升沿：计算完整周期 */
        uint32_t prev_rise = d->rise_tick;
        d->rise_tick = edata->cap_value;
        if (prev_rise != 0) {
            d->period_ticks = d->rise_tick - prev_rise;
        }
    } else {
        /* 下降沿：计算高电平时间 */
        d->high_ticks = edata->cap_value - d->rise_tick;
        if (d->period_ticks > 0) {
            d->data_ready = true;
        }
    }
    return false;   // 不需要唤醒高优先级任务
}

/* ── 从捕获 tick 计算角度 ────────────────────────────────────────────── */
static float compute_angle(uint32_t high_ticks, uint32_t period_ticks)
{
    if (period_ticks == 0) return -1.0f;

    float total   = (float)period_ticks;
    float unit    = total / (float)AS5048A_PWM_PERIOD;
    float valid_h = (float)high_ticks - AS5048A_ZERO_DEG_CLK * unit;
    if (valid_h < 0.0f) return -1.0f;

    float valid_p = total - AS5048A_UNUSED_CLK * unit;
    if (valid_p <= 0.0f) return -1.0f;

    float duty = (valid_h / valid_p) * 100.0f;
    return duty * 3.6f;     // 0-100% → 0-360°
}

/* ── 更新多圈追踪 ────────────────────────────────────────────────────── */
static void update_multiturn(encoder_state_t *enc, float new_raw)
{
    if (new_raw < 0.0f) return;     // 无效读数，跳过

    if (enc->prev_raw >= 0.0f) {
        float delta = new_raw - enc->prev_raw;
        if (delta > 180.0f) {
            enc->turn_count--;      // 350° → 10°：反向跨零
        } else if (delta < -180.0f) {
            enc->turn_count++;      // 10° → 350°：正向跨零
        }
    }
    enc->prev_raw  = new_raw;
    enc->raw_angle = new_raw;
}

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

static float nvs_load_float(const char *key, float default_val)
{
    nvs_handle_t h;
    float val = default_val;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        size_t size = sizeof(float);
        if (nvs_get_blob(h, key, &val, &size) != ESP_OK) {
            val = default_val;
        }
        nvs_close(h);
    }
    return val;
}

/* ── 公共 API ─────────────────────────────────────────────────────────── */

void encoder_hw_init(void)
{
    memset(s_enc, 0, sizeof(s_enc));
    for (int i = 0; i < NUM_JOINTS; i++) {
        s_enc[i].raw_angle = -1.0f;
        s_enc[i].prev_raw  = -1.0f;
    }

    /* 创建 2 个 MCPWM 捕获定时器（每个 group 一个） */
    for (int g = 0; g < 2; g++) {
        mcpwm_capture_timer_config_t timer_cfg = {
            .group_id = g,
            .clk_src  = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        };
        ESP_ERROR_CHECK(mcpwm_new_capture_timer(&timer_cfg, &s_cap_timers[g]));
        ESP_ERROR_CHECK(mcpwm_capture_timer_get_resolution(
            s_cap_timers[g], &s_timer_resolution[g]));
        ESP_LOGI(TAG, "MCPWM group %d capture timer resolution: %"PRIu32" Hz",
                 g, s_timer_resolution[g]);
    }

    /* 为每个关节创建捕获通道 */
    for (int i = 0; i < NUM_JOINTS; i++) {
        int grp = ENCODER_MCPWM[i].group_id;

        mcpwm_capture_channel_config_t ch_cfg = {
            .gpio_num  = JOINT_PINS[i].enc,
            .prescale  = 1,
            .flags = {
                .pos_edge = true,
                .neg_edge = true,
            },
        };
        ESP_ERROR_CHECK(mcpwm_new_capture_channel(
            s_cap_timers[grp], &ch_cfg, &s_enc[i].cap_ch));

        /* 注册 ISR 回调 */
        mcpwm_capture_event_callbacks_t cbs = {
            .on_cap = capture_callback,
        };
        ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(
            s_enc[i].cap_ch, &cbs, &s_enc[i].isr));

        ESP_ERROR_CHECK(mcpwm_capture_channel_enable(s_enc[i].cap_ch));

        ESP_LOGI(TAG, "编码器 %d: GPIO%d → MCPWM%d-CAP%d",
                 i, JOINT_PINS[i].enc, grp, ENCODER_MCPWM[i].cap_channel);
    }

    /* 启动捕获定时器 */
    for (int g = 0; g < 2; g++) {
        ESP_ERROR_CHECK(mcpwm_capture_timer_enable(s_cap_timers[g]));
        ESP_ERROR_CHECK(mcpwm_capture_timer_start(s_cap_timers[g]));
    }

    ESP_LOGI(TAG, "%d 个编码器初始化完成", NUM_JOINTS);
}

float encoder_hw_get_angle(int joint_id)
{
    if (joint_id < 0 || joint_id >= NUM_JOINTS) return 0.0f;

    encoder_state_t *enc = &s_enc[joint_id];

    /* 从 ISR 数据计算原始角度 */
    if (enc->isr.data_ready) {
        float raw = compute_angle(enc->isr.high_ticks, enc->isr.period_ticks);
        update_multiturn(enc, raw);
    }

    /* 有效角度 = 多圈 × 360 + 原始角度 - 零点偏移 */
    if (enc->raw_angle < 0.0f) return 0.0f;
    return enc->turn_count * 360.0f + enc->raw_angle - enc->zero_offset;
}

float encoder_hw_get_raw_angle(int joint_id)
{
    if (joint_id < 0 || joint_id >= NUM_JOINTS) return -1.0f;

    encoder_state_t *enc = &s_enc[joint_id];
    if (enc->isr.data_ready) {
        float raw = compute_angle(enc->isr.high_ticks, enc->isr.period_ticks);
        update_multiturn(enc, raw);
    }
    return enc->raw_angle;
}

void encoder_hw_set_zero(int joint_id)
{
    if (joint_id < 0 || joint_id >= NUM_JOINTS) return;

    encoder_state_t *enc = &s_enc[joint_id];
    enc->zero_offset = enc->raw_angle;
    enc->turn_count  = 0;

    char key[16];
    snprintf(key, sizeof(key), "j%d_zero", joint_id);
    nvs_save_float(key, enc->zero_offset);

    ESP_LOGI(TAG, "关节 %d 零点设定: offset=%.1f°", joint_id, enc->zero_offset);
}

void encoder_hw_load_zeros(void)
{
    for (int i = 0; i < NUM_JOINTS; i++) {
        char key[16];
        snprintf(key, sizeof(key), "j%d_zero", i);
        s_enc[i].zero_offset = nvs_load_float(key, 0.0f);
        ESP_LOGI(TAG, "关节 %d 零点加载: %.1f°", i, s_enc[i].zero_offset);
    }
}
