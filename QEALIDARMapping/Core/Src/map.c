#include "map.h"


volatile struct RobotPos robotPos;

void Init_Robot_Map()
{

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

}

struct NextLoc Find_Next_Loc()
{
    struct NextLoc nextLoc;
    nextLoc.ret = 0;

    //检测周围的障碍,只需要检测前方的就可以
    int fuckCnt = 0;
    for (int i = 0; i < 45; ++i) {
        if (lidarPointData[i].intensity < 30) continue;
        if (lidarPointData[i].distance < 200) fuckCnt++;
    }
    for (int i = 45; i < 90; ++i) {
        if (lidarPointData[i].intensity < 30) continue;
        if (lidarPointData[i].distance < 150) fuckCnt++;
    }
    for (int i = 315; i < 360; ++i) {
        if (lidarPointData[i].intensity < 30) continue;
        if (lidarPointData[i].distance < 200) fuckCnt++;
    }
    for (int i = 270; i < 315; ++i) {
        if (lidarPointData[i].intensity < 30) continue;
        if (lidarPointData[i].distance < 150) fuckCnt++;
    }
    if (fuckCnt < 5) return nextLoc;
    //检测到障碍，选择一条方向避开障碍


//    //寻找梯度最小的方向，下面这个算法容易受干扰，但简单
//    uint16_t minDis = 0;
//    for (int i = 0; i < 360; ++i) {
//        if (lidarPointData[i].intensity < 30) continue;
//        if (lidarPointData[i].distance > minDis) {
//            minDis = lidarPointData[i].distance;
//            nextLoc.ret = 1;
//            nextLoc.radian = lidarPointData[i].radian;
//        }
//    }

    //寻找视野最宽的区域，然后在这个区域去除边界30度内寻找梯度最小的方向。
    int start_angle = -1, end_angle = 360;
    int maxRange = 0;
    int final_start_angle, final_end_angle;
    for (int i = 0; i < 360; ++i) {
        if (lidarPointData[i].intensity < 30) continue;
        if (start_angle == -1) {
            if (lidarPointData[i].distance > 250) {
                start_angle = i;
            }
        }
        if (end_angle == 360) {
            if (lidarPointData[i].distance < 250) {
                end_angle = i;
            }
        }
        if (end_angle != 360) {
            if (end_angle - start_angle > maxRange) {
                maxRange = end_angle -  start_angle;
                final_start_angle = start_angle;
                final_end_angle = end_angle;
            }
            start_angle = -1;
            end_angle = 360;
        }
    }
    if (start_angle != -1) {
        if (end_angle - start_angle > maxRange) {
            maxRange = end_angle -  start_angle;
            final_start_angle = start_angle;
            final_end_angle = end_angle;
        }
    }
    final_start_angle += 30;
    final_end_angle -= 30;
    if (final_start_angle >= final_end_angle) { //范围不足以减去边界，则取中间值
        nextLoc.ret = 1;
        nextLoc.radian = lidarPointData[(final_start_angle + final_end_angle) / 2].radian;
        return nextLoc;
    }
    //找到范围后，在这个范围内寻找梯度最小的方向
    uint16_t minDis = 0;
    for (int i = final_start_angle; i <= final_end_angle; ++i) {
        if (lidarPointData[i].intensity < 30) continue;
        if (lidarPointData[i].distance > minDis) {
            minDis = lidarPointData[i].distance;
            nextLoc.ret = 1;
            nextLoc.radian = lidarPointData[i].radian;
        }
    }

    return nextLoc;
}