#ifndef _my_control_h_
#define _my_control_h_

#include "zf_common_headfile.h"

// ============================================================
// PID 控制器结构体 — 位置式 PID
// ============================================================
typedef struct {
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Kd;               // 微分系数
    float integral;         // 积分累加值
    float last_error;       // 上一次误差（用于微分项计算）
    float integral_limit;   // 积分限幅（抗积分饱和）
    float output_limit;     // 输出限幅
} PID_Controller;

// ============================================================
// 控制参数宏定义（可在此调参）
// ============================================================
#define CTRL_KP                0.8f     // 比例系数
#define CTRL_KI                0.02f    // 积分系数
#define CTRL_KD                0.5f     // 微分系数
#define CTRL_INTEGRAL_LIMIT    30.0f    // 积分限幅
#define CTRL_OUTPUT_LIMIT      100.0f   // 输出限幅（对应 PWM 百分制最大值）
#define CTRL_BASE_SPEED        15       // 基础速度（0~100，百分制占空比）

// ============================================================
// 公开 API
// ============================================================
void control_init(void);     // 初始化 PID 控制器
void control_run(void);       // 每帧/每控制周期调用一次，执行位置式 PID 并输出 PWM 差速

#endif
