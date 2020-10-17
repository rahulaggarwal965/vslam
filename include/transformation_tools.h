#ifndef transformation_tools_h
#define transformation_tools_h

#include <opencv2/core.hpp>

void fundamentalMatrixToRotationTranslation(const cv::Mat& fundamental, cv::Mat& rotation, cv::Mat& translation) {
    float data[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    cv::Mat W(3, 3, CV_32FC1, data);
    cv::Mat U, D, Vt;
    cv::SVD::compute(fundamental, D, U, Vt);
    if (cv::determinant(U) < 0) {
        U *= -1.0;
    }
    if (cv::determinant(Vt) < 0) {
        Vt *= -1.0;
    }
}


#endif
