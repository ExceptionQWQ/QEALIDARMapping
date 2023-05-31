#ifndef MAPPING_H
#define MAPPING_H


#include <opencv2/opencv.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <unistd.h>
#include <memory.h>


extern cv::Mat mappingResult;

void AddToMap(const cv::Mat& frame);


#endif