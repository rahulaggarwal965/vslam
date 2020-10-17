#ifndef transformation_tools_h
#define transformation_tools_h

#include <opencv2/core.hpp>

cv::Mat poseFromRotationTranslation(const cv::Mat& rotation, const cv::Mat& translation);

void fundamentalMatrixToRotationTranslation(const cv::Mat& fundamental, cv::Mat& K, cv::Mat& rotation, cv::Mat& translation);

#endif
