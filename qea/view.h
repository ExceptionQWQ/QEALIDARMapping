#ifndef VIEW_H
#define VIEW_H

#define PI 3.1415926

#include <opencv2/opencv.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <unistd.h>
#include <memory.h>

#include "lidar.h"
#include "robot.h"

extern cv::Mat viewImage;

void View_Start();
cv::Mat GetCloudImage();

#endif