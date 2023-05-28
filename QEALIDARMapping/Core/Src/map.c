#include "map.h"

char slamMap[MAP_X_SIZE][MAP_Y_SIZE];

volatile struct RobotPos robotPos;

void Init_Robot_Map()
{
    for (int x = 0; x < MAP_X_SIZE; ++x) {
        for (int y = 0; y < MAP_Y_SIZE; ++y) {
            slamMap[x][y] = '.';
        }
    }
}

void Init_Robot_Pos()
{
    //居中
    robotPos.x = MAP_X_SIZE / 2;
    robotPos.y = MAP_Y_SIZE / 2;
    robotPos.radian = 2 * PI - robotIMU.heading; //转换成逆时针为正
}


void Update_Map()
{
    robotPos.radian = 2 * PI - robotIMU.heading; //转换成逆时针为正

    for (int angle = 0; angle < 361; ++angle) {
        if (lidarPointData[angle].intensity < 30 || lidarPointData[angle].distance < 100) continue;
        double pointRadian = PI / 180.0 * (360 - angle) ; //转换成逆时针为正
        double radian = robotPos.radian + pointRadian;
        int x = robotPos.x + lidarPointData[angle].distance * cos(radian) / MAP_SIDE_LENGTH;
        int y = robotPos.y + lidarPointData[angle].distance * sin(radian) / MAP_SIDE_LENGTH;
        if (x >= 0 && x < MAP_X_SIZE && y >= 0 && y < MAP_Y_SIZE) {
            slamMap[x][y] = '#';
        }
    }
    ClearLidarData();
}
