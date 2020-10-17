#ifndef transformation_tools_h
#define transformation_tools_h

#include <opencv2/core.hpp>

void fundamentalMatrixToPose(const cv::Mat& fundamental_matrix, const cv::Mat& K, cv::Mat& pose);

#endif
