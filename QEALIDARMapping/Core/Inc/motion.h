#ifndef MOTION_H
#define MOTION_H

#include "main.h"
#include "wheel_pwm.h"
#include "imu.h"
#include "math.h"


volatile struct RobotMotion
{
    double VL;
    double VR;
};

extern volatile struct RobotMotion robotMotion;

void ClearSpeed();
void CommitSpeed();
void MoveForward(double speed);
void MoveBackward(double speed);
void SpinLeft(double speed);
void SpinRight(double speed);
void SpinTo(double radian);




#endif