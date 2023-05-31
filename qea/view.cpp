#include "view.h"

cv::Mat viewImage;

void DrawPoint(cv::Mat mat, double x, double y)
{
    cv::line(mat, cv::Point(x, y), cv::Point(x, y), cv::Scalar(255, 255, 255), 9);
}

cv::Mat GetCloudImage()
{
    cv::Mat image = cv::Mat::zeros(cv::Size(1500, 1500), CV_8UC3);
    for (int i = 0; i < 1200; ++i) {
        double x = robotInfo.xPos + lidarPointData[i].distance * cos((PI / 180 * lidarPointData[i].angle) + robotInfo.heading) * 0.4;
        double y = 1500 - (robotInfo.yPos + lidarPointData[i].distance * sin((PI / 180 * lidarPointData[i].angle) + robotInfo.heading) * 0.4);
        DrawPoint(image, x, y);
    }
    cv::Mat ret;
    cv::resize(image, ret, cv::Size(700, 700), 1, 1);
    return ret;
}

void show()
{
    viewImage = GetCloudImage();
    cv::imshow("view", viewImage);
    cv::waitKey(1);
}

void View_Start()
{
    while (true) {
        show();
    }
}