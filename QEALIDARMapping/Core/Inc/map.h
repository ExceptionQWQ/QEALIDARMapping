#ifndef MAP_H
#define MAP_H

#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "imu.h"
#include "math.h"
#include "lidar.h"

#define PI 3.1415926
#define MAP_X_SIZE 80
#define MAP_Y_SIZE 80
#define MAP_SIDE_LENGTH 10

extern char slamMap[MAP_X_SIZE][MAP_Y_SIZE];



volatile struct RobotPos
{
    uint32_t x; //x坐标
    uint32_t y; //y坐标
    double radian; //弧度 逆时针为正
};


extern volatile struct RobotPos robotPos;


void Init_Robot_Map();
void Init_Robot_Pos();
void Update_Map();


#endif