/*
 * pd_controller.c – PD 位置控制器
 * 含摩擦补偿（最小启动 PWM）和积分项预留（Ki=0 时为纯 PD）。
 */
#include "pd_controller.h"
#include "config.h"

#include <math.h>

/* 钳位函数 */
static inline int clamp_int(int val, int lo, int hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static inline float clamp_float(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

void pd_init(pd_controller_t *pd)
{
    pd->kp             = PD_DEFAULT_KP;
    pd->kd             = PD_DEFAULT_KD;
    pd->ki             = PD_DEFAULT_KI;
    pd->dead_zone      = PD_DEFAULT_DEAD_ZONE;
    pd->min_pwm        = PD_DEFAULT_MIN_PWM;
    pd->prev_error     = 0.0f;
    pd->integral_error = 0.0f;
    pd->integral_limit = PD_DEFAULT_INTEGRAL_LIMIT;
}

int pd_compute(pd_controller_t *pd, float target, float current, float dt)
{
    float error = target - current;

    /* 死区判断：误差足够小则停止输出 */
    if (fabsf(error) <= pd->dead_zone) {
        pd->prev_error     = error;
        pd->integral_error = 0.0f;  // 进入死区时清除积分
        return 0;
    }

    /* 微分项 */
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - pd->prev_error) / dt;
    }

    /* 积分项（Ki > 0 时生效） */
    if (pd->ki > 0.0f) {
        pd->integral_error += error * dt;
        pd->integral_error = clamp_float(
            pd->integral_error, -pd->integral_limit, pd->integral_limit);
    }

    /* PID 输出 */
    float output = pd->kp * error
                 + pd->kd * derivative
                 + pd->ki * pd->integral_error;

    /* 转换为整数 PWM */
    int pwm = (int)output;

    /* 摩擦补偿：误差在死区外但输出太小，提升到最小启动 PWM */
    if (pwm > 0 && pwm < pd->min_pwm) {
        pwm = pd->min_pwm;
    } else if (pwm < 0 && pwm > -pd->min_pwm) {
        pwm = -pd->min_pwm;
    }

    /* 钳位到 ±255 */
    pwm = clamp_int(pwm, -255, 255);

    pd->prev_error = error;
    return pwm;
}

void pd_reset(pd_controller_t *pd)
{
    pd->prev_error     = 0.0f;
    pd->integral_error = 0.0f;
}
