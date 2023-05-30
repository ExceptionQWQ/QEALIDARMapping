#include "detect_circle.h"


struct Get_Circle_Result Get_Circle(double x1, double y1, double x2, double y2, double x3, double y3)
{
    struct Get_Circle_Result getCircleResult;
    double a = 2 * (x2 - x1);
    double b = 2 * (y2 - y1);
    double c = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1;
    double d = 2 * (x3 - x2);
    double e = 2 * (y3 - y2);
    double f = x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2;
    getCircleResult.x = (b * f - e * c) / (b * d - e * a);
    getCircleResult.y = (d * c - a * f) / (b * d - e * a);
    getCircleResult.radius = sqrt((getCircleResult.x - x1) * (getCircleResult.x - x1) + (getCircleResult.y - y1) * (getCircleResult.y - y1));
    return getCircleResult;
}

struct Detect_Circle_Result Ransac_Circles(int minR, int maxR)
{
    struct Detect_Circle_Result detectCircleResult = {};
    int maxCnt = 0;

    //按0-360的顺序连续选取3个点当作圆上的3点
    for (int angle = 0; angle < 370; angle += 1) {
        if (lidarPointData[angle % 360].intensity < 30) continue;
        if (lidarPointData[(angle + 2) % 360].intensity < 30) continue;
        if (lidarPointData[(angle + 4) % 360].intensity < 30) continue;
        if (lidarPointData[angle % 360].distance < 300) continue;
        if (lidarPointData[(angle + 2) % 360].distance < 300) continue;
        if (lidarPointData[(angle + 4) % 360].distance < 300) continue;

        double x1 = lidarPointData[angle % 360].x, y1 = lidarPointData[angle % 360].y;
        double x2 = lidarPointData[(angle + 5) % 360].x, y2 = lidarPointData[(angle + 5) % 360].y;
        double x3 = lidarPointData[(angle + 10) % 360].x, y3 = lidarPointData[(angle + 10) % 360].y;
        struct Get_Circle_Result getCircleResult = Get_Circle(x1, y1, x2, y2, x3, y3);
        if (getCircleResult.radius < minR || getCircleResult.radius > maxR) continue;;
        int cnt = 0; //计算圆上点的个数
        for (int i = 0; i < 360; ++i) {
            if (lidarPointData[i].intensity < 30) continue;
            double r = sqrt(pow(lidarPointData[i].x - getCircleResult.x, 2) + pow(lidarPointData[i].y - getCircleResult.y, 2));
            if (fabs(r - getCircleResult.radius) < 10) ++cnt;
        }
        if (cnt > maxCnt) {
            maxCnt = cnt;
            detectCircleResult.thresh = maxCnt;
            detectCircleResult.radius = getCircleResult.radius;
            detectCircleResult.x = getCircleResult.x;
            detectCircleResult.y = getCircleResult.y;
        }
    }

    return detectCircleResult;
}
