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
 * 使用方式：
 *   在主循环中（如 50Hz 定时中断），每帧调用一次 control_run()。
 *   调用前请确保 ProcessFrame() 已执行完毕，Dir_err 为最新值。
 */

#include "my_control.h"
#include "my_line_follow.h"
#include "my_motor.h"
#include "my_position.h"

// ============================================================
// 全局 PID 控制器实例
// ============================================================
static PID_Controller pid_dir;

// ============================================================
// control_init — 初始化 PID 参数与状态
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

    // ------ Step 2: 位置式 PID 计算差速量 ------ 
    float diff_pwm = PID_Positional(&pid_dir, error);

    // ------ Step 3: 分配左右轮速度 ------ 
    // 约定：motor_a = 左轮，motor_b = 右轮
    // Dir_err > 0（中线偏左）→ 需左转 → 右轮加速 / 左轮减速
    int16 left_speed  = (int16)((float)CTRL_BASE_SPEED - diff_pwm / 2.0f);
    int16 right_speed = (int16)(((float)CTRL_BASE_SPEED + diff_pwm / 2.0f)*0.7);

    // ------ Step 4: 输出 PWM 到电机 ------ 
    motor_a_set(left_speed);    // 左轮
    motor_b_set(right_speed);   // 右轮
}
