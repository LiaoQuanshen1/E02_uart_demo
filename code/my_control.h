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
// 控制参数（可被菜单实时修改，初始值见 my_control.c）
// ============================================================
extern float CTRL_KP;               // 比例系数
extern float CTRL_KI;               // 积分系数
extern float CTRL_KD;               // 微分系数
extern float CTRL_INTEGRAL_LIMIT;   // 积分限幅
extern float CTRL_OUTPUT_LIMIT;     // 输出限幅（对应 PWM 百分制最大值）
extern int   CTRL_BASE_SPEED;       // 基础速度（0~100，百分制占空比）
extern float CTRL_ADJUST;           // 调整系数

// ============================================================
// 公开 API
// ============================================================
void control_init(void);     // 初始化 PID 控制器
void control_run(void);       // 每帧/每控制周期调用一次，执行位置式 PID 并输出 PWM 差速
void control_test(void);      // 直驱测试
void menu_setup_ctrl(void);   // 创建控制参数菜单（由 main.c 的 menu_setup 调用）

#endif
