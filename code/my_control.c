/**
 * my_control.c — 位置式 PID 方向控制器
 *
 * 功能说明：
 *   以图像处理模块（my_line_follow.c）输出的方向偏差 Dir_err 作为 PID 输入，
 *   计算左右轮 PWM 差速值，驱动电机实现巡线转向。
 *
 * 坐标系与符号约定：
 *   Dir_err > 0  → 赛道中线在图像中心左侧 → 车应左转
 *   Dir_err < 0  → 赛道中线在图像中心右侧 → 车应右转
 *
 * 差速策略（以 Dir_err > 0 为例）：
 *   left_speed  = base_speed - pid_output / 2  （左轮减速）
 *   right_speed = base_speed + pid_output / 2  （右轮加速）
 *   → 车体向左转向，追踪赛道中线
 *
 *   假设 motor_a = 左轮，motor_b = 右轮（如实际接线相反，请交换赋值）。
 *
 * 位置式 PID 公式：
 *   output(t) = Kp·e(t) + Ki·Σe(t) + Kd·[e(t) - e(t-1)]
 *
 */

#include "my_control.h"
#include "my_line_follow.h"
#include "my_motor.h"
#include "my_position.h"
#include "zf_components_menu.h"
#include "zf_driver_pit.h"
#include "zf_driver_uart.h"
#include "stdio.h"

// ============================================================
// 可调参数 — 由菜单实时修改（初始值 = 原宏定义默认值）
// ============================================================
float CTRL_KP               = 1.0f;
float CTRL_KI               = 0.02f;
float CTRL_KD               = 0.5f;
float CTRL_INTEGRAL_LIMIT   = 30.0f;
float CTRL_OUTPUT_LIMIT     = 100.0f;
int   CTRL_BASE_SPEED       = 10;
float CTRL_ADJUST           = 0.7f;

// ============================================================
// 全局 PID 控制器实例
// ============================================================
static PID_Controller pid_dir;

// ============================================================
// control_init — 初始化 PID 参数 + DWT 周期计数器
// ============================================================
void control_init(void)
{
    pid_dir.Kp             = CTRL_KP;
    pid_dir.Ki             = CTRL_KI;
    pid_dir.Kd             = CTRL_KD;
    pid_dir.integral       = 0.0f;
    pid_dir.last_error     = 0.0f;
    pid_dir.integral_limit = CTRL_INTEGRAL_LIMIT;
    pid_dir.output_limit   = CTRL_OUTPUT_LIMIT;

    // 使能 DWT 周期计数器（用于 ISR 耗时测量）
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// ============================================================
// PID_Positional — 位置式 PID 核心算法
//
// 参数：
//   pid   — PID 控制器指针
//   error — 当前误差 = Dir_err（来自图像处理模块）
//
// 返回：
//   经过限幅的 PID 输出值（即左右轮 PWM 差速量）
//
// 抗积分饱和：当积分项超过 integral_limit 时做截断处理。
// ============================================================
static float PID_Positional(PID_Controller *pid, float error)
{
    // ------ 1. 比例项 P ------ 
    float P = pid->Kp * error;

    // ------ 2. 积分项 I（含抗积分饱和）------ 
    pid->integral += error;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float I = pid->Ki * pid->integral;

    // ------ 3. 微分项 D ------ 
    float D = pid->Kd * (error - pid->last_error);
    pid->last_error = error;

    // ------ 4. 合成输出并限幅 ------ 
    float output = P + I + D;
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    return output;
}

// ============================================================
// control_run — 控制主循环（每控制周期调用一次）
//
// 执行流程：
//   1. 读取图像处理模块输出的 Dir_err
//   2. 位置式 PID 计算差速量 diff_pwm
//   3. 将差速量分配到左右轮并输出 PWM
//
// 注意：
//   motor_a_set / motor_b_set 的参数范围为 -100 ~ +100（百分制占空比），
//   正值为正转，负值为反转。motor_set 内部已有限幅。
//   若实际接线 motor_a 对应右轮，请交换下方的 left/right 赋值。
// ============================================================
void control_run(void)
{
    

    // ------ Step 1: 获取方向误差 ------ 
    float error = Dir_err;   // 来自 my_line_follow.c，全局变量

    // ------ Step 1.5: 同步菜单修改的 PID 参数到运行时控制器 ------ 
    pid_dir.Kp             = CTRL_KP;
    pid_dir.Ki             = CTRL_KI;
    pid_dir.Kd             = CTRL_KD;
    pid_dir.integral_limit = CTRL_INTEGRAL_LIMIT;
    pid_dir.output_limit   = CTRL_OUTPUT_LIMIT;

    // ------ Step 2: 位置式 PID 计算差速量 ------ 
    float diff_pwm = PID_Positional(&pid_dir, error);

    // ------ Step 3: 分配左右轮速度 ------ 
    // 约定：motor_a = 左轮，motor_b = 右轮
    // Dir_err > 0（中线偏左）→ 需左转 → 右轮加速 / 左轮减速
    int16 left_speed  = (int16)((float)CTRL_BASE_SPEED - diff_pwm / 2.0f);
    int16 right_speed = (int16)(((float)CTRL_BASE_SPEED + diff_pwm / 2.0f)*CTRL_ADJUST);
    // ------ Step 0: 丢线保护 —— 截止行靠近底部 → 立即停车 ------ 
    // imgTop 越大可见越近，LINE_IMG_H-1=119 为图像最底行
    if (imgTop > LINE_IMG_H - 10)                               // 最远可见行距底部不足 10 像素
    {
        motor_a_set(0);
        motor_b_set(0);
        return;
    }
    // ------ Step 4: 输出 PWM 到电机 ------ 
    motor_a_set(left_speed);    // 左轮
    motor_b_set(right_speed);   // 右轮
    
}
void control_test(void)
{
    motor_a_set(CTRL_BASE_SPEED);
    motor_b_set((int16)(CTRL_BASE_SPEED*CTRL_ADJUST));
}

// ============================================================
// 控制参数菜单（供 main.c 调用，参数集中于本模块管理）
// ============================================================

static MENU_ITEM m_ctrl_root;
static MENU_ITEM m_ctrl_kp, m_ctrl_ki, m_ctrl_kd, m_ctrl_intlim, m_ctrl_outlim, m_ctrl_basespd, m_ctrl_adjust;

static param_desc_t p_ctrl_kp       = { &CTRL_KP,             float_Box, 0.1f,  0.0f,  10.0f  };
static param_desc_t p_ctrl_ki       = { &CTRL_KI,             float_Box, 0.01f, 0.0f,   1.0f  };
static param_desc_t p_ctrl_kd       = { &CTRL_KD,             float_Box, 0.1f,  0.0f,   5.0f  };
static param_desc_t p_ctrl_intlim   = { &CTRL_INTEGRAL_LIMIT, float_Box, 5.0f,  5.0f, 100.0f  };
static param_desc_t p_ctrl_outlim   = { &CTRL_OUTPUT_LIMIT,   float_Box, 5.0f, 20.0f, 100.0f  };
static param_desc_t p_ctrl_basespd  = { &CTRL_BASE_SPEED,     int_Box,   5,     0,    100     };
static param_desc_t p_ctrl_adjust   = { &CTRL_ADJUST,         float_Box, 0.05f, 0.5f,   1.5f  };

void menu_setup_ctrl(void)
{
    Create_Menu_Folder(&head,      &m_ctrl_root,    "Control");
    Create_Menu_Number(&m_ctrl_root, &m_ctrl_kp,      "Kp",       &p_ctrl_kp);
    Create_Menu_Number(&m_ctrl_root, &m_ctrl_ki,      "Ki",       &p_ctrl_ki);
    Create_Menu_Number(&m_ctrl_root, &m_ctrl_kd,      "Kd",       &p_ctrl_kd);
    Create_Menu_Number(&m_ctrl_root, &m_ctrl_intlim,  "IntLim",   &p_ctrl_intlim);
    Create_Menu_Number(&m_ctrl_root, &m_ctrl_outlim,  "OutLim",   &p_ctrl_outlim);
    Create_Menu_Number(&m_ctrl_root, &m_ctrl_basespd, "BaseSpd",  &p_ctrl_basespd);
    Create_Menu_Number(&m_ctrl_root, &m_ctrl_adjust,  "Adjust",   &p_ctrl_adjust);
}
