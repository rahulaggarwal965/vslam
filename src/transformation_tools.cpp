#include "transformation_tools.h"

void fundamentalMatrixToPose(const cv::Mat& fundamental_matrix, const cv::Mat& K, cv::Mat& pose) {
    cv::Mat K_t;
    cv::transpose(K, K_t);
    cv::Mat essential = K_t * fundamental_matrix * K;
    //TODO: investigate Recover Pose

    float data[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    cv::Mat W(3, 3, CV_32FC1, data);
    cv::Mat U, D, Vt;
    cv::SVD::compute(essential, D, U, Vt);
    if (cv::determinant(U) < 0) {
        U *= -1.0;
    }
    if (cv::determinant(Vt) < 0) {
        Vt *= -1.0;
    }
    pose = cv::Mat::eye(4, 4, CV_32FC1);
    //TODO: maybe bug
    pose(cv::Range(0, 3), cv::Range(0, 3)) = U * W * Vt;
    pose(cv::Range(0, 3), cv::Range(3, 4))  = U.col(2).clone();
}
