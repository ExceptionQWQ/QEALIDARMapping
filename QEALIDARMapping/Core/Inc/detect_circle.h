#ifndef DETECT_CIRCLE_H
#define DETECT_CIRCLE_H

#include "main.h"
#include "math.h"
#include "lidar.h"


struct Detect_Circle_Result
{
    int x;
    int y;
    int thresh;
    int32_t radius;
};


struct Get_Circle_Result
{
    double x;
    double y;
    double radius;
};

/*
 * 检测圆，返回圆心坐标，没有则返回半径为0的圆
 * @param minR 最小检测半径 maxR 最大检测半径
 */
struct Detect_Circle_Result Ransac_Circles(int minR, int maxR);



#endif