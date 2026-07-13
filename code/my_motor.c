#include "my_motor.h"

// 引脚定义（来自推荐引脚分配：TIM5 2路PWM + 2路IO）
#define MOTOR_A_DIR  A0                                                        // 电机A 方向
#define MOTOR_A_PWM  TIM5_PWM_CH2_A1                                           // 电机A 速度 PWM
#define MOTOR_B_DIR  A2                                                        // 电机B 方向
#define MOTOR_B_PWM  TIM5_PWM_CH4_A3                                           // 电机B 速度 PWM
#define PWM_FREQ     10000                                                     // PWM 频率 10kHz

// 电机速度控制参数（百分制输入 → 内部 duty 值换算）
#define MOTOR_SPEED_MAX     100                                                // 速度输入范围：-100 ~ +100（百分制）
#define MOTOR_DUTY_SCALE    (PWM_DUTY_MAX / MOTOR_SPEED_MAX)                   // 百分制→duty 换算系数
#define MOTOR_CLAMP_RATE(r)  ((r) > MOTOR_SPEED_MAX ? MOTOR_SPEED_MAX :        \
                              (r) < -MOTOR_SPEED_MAX ? -MOTOR_SPEED_MAX : (r)) // 限幅到 [-MAX, +MAX]

void motor_init (void)
{
    gpio_init(MOTOR_A_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_B_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_A_PWM, PWM_FREQ, 0);
    pwm_init(MOTOR_B_PWM, PWM_FREQ, 0);
}

static void motor_set (gpio_pin_enum dir, pwm_channel_enum ch, int16 rate)
{
    uint16 duty;                                                                // 待写入的占空比（0 ~ PWM_DUTY_MAX）

    rate = MOTOR_CLAMP_RATE(rate);                                              // 限幅到 [-MOTOR_SPEED_MAX, +MOTOR_SPEED_MAX]

    if(rate > 0)
    {
        gpio_high(dir);                                                         // 正向：方向引脚输出高电平
        duty = (uint16)rate * MOTOR_DUTY_SCALE;
        pwm_set_duty(ch, duty);
    }
    else if(rate < 0)
    {
        gpio_low(dir);                                                          // 反向：方向引脚输出低电平
        duty = (uint16)(-rate) * MOTOR_DUTY_SCALE;
        pwm_set_duty(ch, duty);//正转或者反转，自己进行尝试，有可能会后续修改
    }
    else
    {
        pwm_set_duty(ch, 0);                                                    // 停止：占空比清零
    }
}

void motor_a_set (int16 rate) { motor_set(MOTOR_A_DIR, MOTOR_A_PWM, rate); }
void motor_b_set (int16 rate) { motor_set(MOTOR_B_DIR, MOTOR_B_PWM, rate); }
