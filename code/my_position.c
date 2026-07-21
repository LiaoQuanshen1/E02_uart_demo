#include "my_position.h"
#include <math.h>

float pitch_angle = 0.0f;
float roll_angle  = 0.0f;

void my_position_update(void)
{
    imu660ra_get_acc();
    imu660ra_get_gyro();

    float ax = imu660ra_acc_transition(imu660ra_acc_x);
    float ay = imu660ra_acc_transition(imu660ra_acc_y);
    float az = imu660ra_acc_transition(imu660ra_acc_z);
    float gx = imu660ra_gyro_transition(imu660ra_gyro_x);
    float gy = imu660ra_gyro_transition(imu660ra_gyro_y);

    float acc_pitch = atan2f(ax, ay) * 57.29578f;//?????
    float acc_roll  = atan2f(ax, az) * 57.29578f;//?????

    // »¥²¹ÂË²¨£º¦Á=0.96 ÐÅÈÎÍÓÂÝÒÇ£¬dt=0.02(50HzÖ÷Ñ­»·)
    pitch_angle = 0.96f * (pitch_angle + gy * 0.02f) + 0.04f * acc_pitch;
    roll_angle  = 0.96f * (roll_angle  + gx * 0.02f) + 0.04f * acc_roll;
}
