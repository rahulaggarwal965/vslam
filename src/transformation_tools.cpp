#include "transformation_tools.h"

//TODO: refactor
void poseFromRotationTranslation(const cv::Mat& rotation, const cv::Mat& translation, cv::Mat& pose) {
    pose = cv::Mat::eye(4, 4, CV_32FC1);
    rotation.copyTo(pose(cv::Rect(cv::Point(0, 0), rotation.size())));
    pose.col(3) = translation;
}

void fundamentalMatrixToRotationTranslation(const cv::Mat& fundamental, cv::Mat& K, cv::Mat& rotation, cv::Mat& translation) {
    cv::Mat K_t;
    cv::transpose(K, K_t);
    cv::Mat essential = K_t * fundamental * K;
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
    rotation = U * W * Vt;
    translation = U.col(2).clone();
}
