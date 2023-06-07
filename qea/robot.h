#ifndef ROBOT_H
#define ROBOT_H

#define PI 3.1415926

#include <iostream>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <unistd.h>
#include <memory.h>
#include <wiringSerial.h>


#define ROBOT_DEV_PATH "/dev/ttyACM0"
#define ROBOT_BAUD 115200
// #define ROBOT_DIS 0.027
#define ROBOT_DIS 0

struct RobotInfo
{
    double xPos;
    double yPos;
    double roll;
    double pitch;
    double heading;
};

extern struct RobotInfo robotInfo;

extern int robotSerial;

extern char robotRecvBuff[1024];
extern int robotRecvOffset;

void Robot_Init();
void Robot_Start();

void MoveForward(int speed, int delay);
void MoveBackward(int speed, int delay);
void SpinTo(double radian);
void RobotStop();



#endif