#ifndef _my_position_h_
#define _my_position_h_

#include "zf_common_headfile.h"
#define alpha 0.96f // 互补滤波系数，0.96 信任陀螺仪，0.04 信任加速度计
extern float pitch_angle;    // 俯仰角（°），正值=上坡
extern float roll_angle;     // 横滚角（°）

void my_position_update(void);

#endif
