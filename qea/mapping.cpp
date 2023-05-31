#include "mapping.h"

cv::Mat mappingResult;

void AddToMap(const cv::Mat& frame)
{
    if (mappingResult.empty()) {
        mappingResult = frame;
        cv::imshow("mappingResult", mappingResult);
        cv::waitKey(1);
        return ;
    }
    cv::bitwise_or(mappingResult, frame, mappingResult);
    cv::imshow("mappingResult", mappingResult);
    cv::waitKey(1);

    return ;
}