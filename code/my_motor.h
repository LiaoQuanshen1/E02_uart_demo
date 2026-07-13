#ifndef _my_motor_h_
#define _my_motor_h_

#include "zf_common_headfile.h"

void motor_init  (void);
void motor_a_set (int16 speed);                                                // speed: -100 ~ +100（百分制占空比），负值反转
void motor_b_set (int16 speed);

#endif
