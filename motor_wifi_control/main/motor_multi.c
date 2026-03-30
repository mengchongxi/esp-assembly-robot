/*
 * motor_multi.c – 5 路 LEDC PWM 电机驱动
 * 使用 ESP32 LEDC（LED PWM 控制器）驱动 5 个直流电机。
 * 低速模式 8 通道 + 高速模式 2 通道 = 10 路 PWM。
 */
#include "motor_multi.h"
#include "config.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "motor";

/* ── LEDC 通道映射表 ──────────────────────────────────────────────────── */
/* 每个关节 2 个通道（IN1, IN2），按顺序分配 */
typedef struct {
    ledc_mode_t    speed_mode;
    ledc_timer_t   timer;
    ledc_channel_t ch_in1;
    ledc_channel_t ch_in2;
} motor_ledc_t;

static const motor_ledc_t MOTOR_LEDC[NUM_JOINTS] = {
    /* 关节1: 低速 Timer0, CH0/CH1 */
    { LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, LEDC_CHANNEL_0, LEDC_CHANNEL_1 },
    /* 关节2: 低速 Timer0, CH2/CH3 */
    { LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, LEDC_CHANNEL_2, LEDC_CHANNEL_3 },
    /* 关节3: 低速 Timer1, CH4/CH5 */
    { LEDC_LOW_SPEED_MODE, LEDC_TIMER_1, LEDC_CHANNEL_4, LEDC_CHANNEL_5 },
    /* 关节4: 低速 Timer1, CH6/CH7（原spec中写的 LEDC_CHANNEL_6 和 LEDC_CHANNEL_7） */
    { LEDC_LOW_SPEED_MODE, LEDC_TIMER_1, LEDC_CHANNEL_6, LEDC_CHANNEL_7 },
    /* 关节5: 高速 Timer0, CH0/CH1 */
    { LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, LEDC_CHANNEL_0, LEDC_CHANNEL_1 },
};

/* ── 内部辅助 ─────────────────────────────────────────────────────────── */
static void set_duty(ledc_mode_t mode, ledc_channel_t ch, uint32_t duty)
{
    ledc_set_duty(mode, ch, duty);
    ledc_update_duty(mode, ch);
}

/* ── 公共 API ─────────────────────────────────────────────────────────── */

void motor_multi_init(void)
{
    /* 配置低速模式 Timer0 */
    ledc_timer_config_t ls_timer0 = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ls_timer0));

    /* 配置低速模式 Timer1 */
    ledc_timer_config_t ls_timer1 = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .timer_num       = LEDC_TIMER_1,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ls_timer1));

    /* 配置高速模式 Timer0 */
    ledc_timer_config_t hs_timer0 = {
        .speed_mode      = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&hs_timer0));

    /* 为每个关节配置 2 个 LEDC 通道 */
    for (int i = 0; i < NUM_JOINTS; i++) {
        const motor_ledc_t *ml = &MOTOR_LEDC[i];

        ledc_channel_config_t ch_in1 = {
            .gpio_num   = JOINT_PINS[i].in1,
            .speed_mode = ml->speed_mode,
            .channel    = ml->ch_in1,
            .timer_sel  = ml->timer,
            .duty       = 0,
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_in1));

        ledc_channel_config_t ch_in2 = {
            .gpio_num   = JOINT_PINS[i].in2,
            .speed_mode = ml->speed_mode,
            .channel    = ml->ch_in2,
            .timer_sel  = ml->timer,
            .duty       = 0,
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_in2));

        ESP_LOGI(TAG, "电机 %d: GPIO%d/GPIO%d → %s Timer%d CH%d/CH%d",
                 i, JOINT_PINS[i].in1, JOINT_PINS[i].in2,
                 ml->speed_mode == LEDC_LOW_SPEED_MODE ? "LS" : "HS",
                 ml->timer, ml->ch_in1, ml->ch_in2);
    }

    ESP_LOGI(TAG, "%d 路电机初始化完成", NUM_JOINTS);
}

void motor_multi_set_speed(int joint_id, int speed)
{
    if (joint_id < 0 || joint_id >= NUM_JOINTS) return;

    if (speed >  255) speed =  255;
    if (speed < -255) speed = -255;

    const motor_ledc_t *ml = &MOTOR_LEDC[joint_id];

    if (speed > 0) {
        set_duty(ml->speed_mode, ml->ch_in1, (uint32_t)speed);
        set_duty(ml->speed_mode, ml->ch_in2, 0);
    } else if (speed < 0) {
        set_duty(ml->speed_mode, ml->ch_in1, 0);
        set_duty(ml->speed_mode, ml->ch_in2, (uint32_t)(-speed));
    } else {
        set_duty(ml->speed_mode, ml->ch_in1, 0);
        set_duty(ml->speed_mode, ml->ch_in2, 0);
    }
}

void motor_multi_stop(int joint_id)
{
    motor_multi_set_speed(joint_id, 0);
}

void motor_multi_stop_all(void)
{
    for (int i = 0; i < NUM_JOINTS; i++) {
        motor_multi_stop(i);
    }
    ESP_LOGW(TAG, "所有电机已急停");
}
