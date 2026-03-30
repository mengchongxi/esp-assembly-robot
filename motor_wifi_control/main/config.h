#pragma once

#include <stdint.h>

// ── 关节数量 ──────────────────────────────────────────────────────────────
#define NUM_JOINTS  5

// ── 关节引脚映射 ──────────────────────────────────────────────────────────
// 索引 0-4 对应关节 1-5
// 关节1=CN2(U3-B), 关节2=CN5(U4-B), 关节3=CN6(U4-A),
// 关节4=CN7(U7-A), 关节5=CN4(U3-A)
typedef struct {
    int in1;    // H 桥 IN1 引脚（正转 PWM）
    int in2;    // H 桥 IN2 引脚（反转 PWM）
    int enc;    // AS5048A 编码器 PWM 输入引脚
} joint_pin_t;

static const joint_pin_t JOINT_PINS[NUM_JOINTS] = {
    { .in1 =  4, .in2 = 13, .enc = 36 },   // 关节1: CN2, U3-B, SERVO1
    { .in1 = 18, .in2 = 19, .enc = 39 },   // 关节2: CN5, U4-B, SERVO2
    { .in1 = 16, .in2 = 17, .enc = 34 },   // 关节3: CN6, U4-A, SERVO3
    { .in1 = 23, .in2 = 22, .enc = 35 },   // 关节4: CN7, U7-A, SERVO4
    { .in1 = 27, .in2 = 14, .enc = 21 },   // 关节5: CN4, U3-A, SERVO5
};

// ── LEDC PWM 设置 ─────────────────────────────────────────────────────────
#define MOTOR_PWM_FREQ_HZ       20000
#define MOTOR_PWM_RESOLUTION    8           // 8-bit → duty 0-255

// ── MCPWM Capture 编码器分配 ──────────────────────────────────────────────
// Group 0: CAP0=GPIO36, CAP1=GPIO39, CAP2=GPIO34 (关节 1,2,3)
// Group 1: CAP0=GPIO35, CAP1=GPIO21           (关节 4,5)
typedef struct {
    int group_id;
    int cap_channel;    // 0, 1, 2
} encoder_mcpwm_t;

static const encoder_mcpwm_t ENCODER_MCPWM[NUM_JOINTS] = {
    { .group_id = 0, .cap_channel = 0 },   // 关节1: Group0-CAP0
    { .group_id = 0, .cap_channel = 1 },   // 关节2: Group0-CAP1
    { .group_id = 0, .cap_channel = 2 },   // 关节3: Group0-CAP2
    { .group_id = 1, .cap_channel = 0 },   // 关节4: Group1-CAP0
    { .group_id = 1, .cap_channel = 1 },   // 关节5: Group1-CAP1
};

// ── AS5048A PWM 协议常数 ──────────────────────────────────────────────────
#define AS5048A_PWM_PERIOD          4119
#define AS5048A_ZERO_DEG_CLK        16
#define AS5048A_EXIT_CLK            8
#define AS5048A_UNUSED_CLK          (AS5048A_ZERO_DEG_CLK + AS5048A_EXIT_CLK)

// ── PD 控制默认参数 ───────────────────────────────────────────────────────
#define PD_DEFAULT_KP               2.0f
#define PD_DEFAULT_KD               0.5f
#define PD_DEFAULT_KI               0.0f
#define PD_DEFAULT_DEAD_ZONE        1.0f    // 度
#define PD_DEFAULT_MIN_PWM          20
#define PD_DEFAULT_INTEGRAL_LIMIT   1000.0f

// ── 关节默认限位 ──────────────────────────────────────────────────────────
#define JOINT_DEFAULT_MIN_LIMIT     (-360.0f)
#define JOINT_DEFAULT_MAX_LIMIT     (360.0f)

// ── 控制任务参数 ──────────────────────────────────────────────────────────
#define CONTROL_TASK_PRIORITY       5
#define CONTROL_TASK_STACK_SIZE     4096
#define CONTROL_LOOP_PERIOD_MS      10      // 100 Hz

// ── NVS 命名空间 ─────────────────────────────────────────────────────────
#define NVS_NAMESPACE               "joints"

// ── WiFi ──────────────────────────────────────────────────────────────────
#define WIFI_SSID           "chonxi"
#define WIFI_PASSWORD       "12345678"
#define WIFI_MAX_RETRY      10

// ── HTTP 服务器 ───────────────────────────────────────────────────────────
#define HTTP_SERVER_PORT    80
