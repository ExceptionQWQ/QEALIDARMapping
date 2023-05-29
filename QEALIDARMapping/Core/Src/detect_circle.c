#include "detect_circle.h"

uint8_t hough_trans[HOUGH_X_SIZE][HOUGH_Y_SIZE];

void Clear_Hough_Result()
{
    for (int i = 0; i < HOUGH_X_SIZE; ++i) {
        for (int j = 0; j < HOUGH_Y_SIZE; ++j) {
            hough_trans[i][j] = 0;
        }
    }
}

struct Find_Max_Result Find_Max_Loc()
{
    uint8_t maxLoc = 0;
    struct Find_Max_Result findMaxResult = {};
    for (int i = 0; i < HOUGH_X_SIZE; ++i) {
        for (int j = 0; j < HOUGH_Y_SIZE; ++j) {
            if (hough_trans[i][j] > maxLoc) {
                maxLoc = hough_trans[i][j];
                findMaxResult.x = i;
                findMaxResult.y = j;
            }
        }
    }
    return findMaxResult;
}


struct Detect_Circle_Result Hough_Circles(uint8_t threshold, int32_t radius)
{
    Clear_Hough_Result();
    struct Detect_Circle_Result detectCircleResult;
    for (int angle = 0; angle < 360; ++angle) { //枚举每个点,把它当作圆心
        if (lidarPointData[angle].intensity < 30) continue;
        if (lidarPointData[angle].distance > 2000) continue;
        for (int circle_angle = 0; circle_angle < 360; circle_angle += HOUGH_CIRCLE_ENUM_STEP) {
            double x = lidarPointData[angle].x;
            double y = lidarPointData[angle].y;
            x += radius * cos(PI / 180 * circle_angle);
            y += radius * sin(PI / 180 * circle_angle);
            x /= HOUGH_CIRCLE_SIDE_LEN;
            y /= HOUGH_CIRCLE_SIDE_LEN;
            x += HOUGH_X_CENTER;
            y += HOUGH_Y_CENTER;
            int tx = x, ty = y;
            if (tx >= 0 && tx < HOUGH_X_SIZE && ty >= 0 && ty < HOUGH_Y_SIZE) {
                hough_trans[tx][ty] += 1;
            }
        }
    }
    struct Find_Max_Result findMaxResult = Find_Max_Loc();
    detectCircleResult.thresh = hough_trans[findMaxResult.x][findMaxResult.y];
    detectCircleResult.x = (findMaxResult.x - HOUGH_X_CENTER) * HOUGH_CIRCLE_SIDE_LEN;
    detectCircleResult.y = (findMaxResult.y - HOUGH_Y_CENTER) * HOUGH_CIRCLE_SIDE_LEN;
    if (hough_trans[findMaxResult.x][findMaxResult.y] < threshold) {
        detectCircleResult.radius = 0;
    } else {
        detectCircleResult.radius = radius;
    }
    return detectCircleResult;
}