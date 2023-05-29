#ifndef DETECT_CIRCLE_H
#define DETECT_CIRCLE_H

#include "main.h"
#include "math.h"
#include "lidar.h"

#define HOUGH_X_SIZE 90
#define HOUGH_Y_SIZE 90
#define HOUGH_X_CENTER 0
#define HOUGH_Y_CENTER 45
#define HOUGH_CIRCLE_ENUM_STEP 10
#define HOUGH_CIRCLE_SIDE_LEN 10

struct Detect_Circle_Result
{
    int x;
    int y;
    int thresh;
    int32_t radius;
};


struct Find_Max_Result
{
    int x;
    int y;
};

extern uint8_t hough_trans[HOUGH_X_SIZE][HOUGH_Y_SIZE];

void Clear_Hough_Result();
/*
 * 检测圆，返回圆心坐标，没有则返回半径为0的圆
 * @param threshold 阈值，大于这个阈值说明这个是圆 radius 要检测的圆的半径
 */
struct Detect_Circle_Result Hough_Circles(uint8_t threshold, int32_t radius);

/*
 * 寻找最大值的位置
 */
struct Find_Max_Result Find_Max_Loc();


#endif