#include "transformation_tools.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/types.hpp"
#include <iostream>

#define MAX_ITERATIONS 30

void triangulate(const cv::Mat &pose1, const cv::Mat& pose2, const cv::Point2f &p1, const cv::Point2f &p2, cv::Mat& r3d) {
    cv::Mat E(4, 4, CV_32FC1);

    E.row(0) = p1.x*pose1.row(2)-pose1.row(0);
    E.row(1) = p1.y*pose1.row(2)-pose1.row(1);
    E.row(2) = p2.x*pose2.row(2)-pose2.row(0);
    E.row(3) = p2.y*pose2.row(2)-pose2.row(1);

    cv::Mat U, D, V_t;
    cv::SVDecomp(E, D, U, V_t, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    r3d = V_t.row(3).t();
    r3d = r3d.rowRange(0, 3)/r3d.at<float>(3);

}

void poseRt(const cv::Mat &R, const cv::Mat &t, cv::Mat &pose) {
  pose = cv::Mat::eye(4, 4, CV_32F);
  R.copyTo(pose.rowRange(0, 3).colRange(0, 3));
  t.copyTo(pose.rowRange(0, 3).col(3));
}

void fundamentalMatrixToRt(const cv::Mat& fundamental_matrix, const cv::Mat& K, cv::Mat& R, cv::Mat& t) {
    std::cout << "Fundamental Matrix: " << "\n" << fundamental_matrix << "\n" << "\n";
    cv::Mat F;
    fundamental_matrix.convertTo(F, CV_32FC1);
    cv::Mat essential = K.t()* F * K;

    cv::Mat U, D, Vt;
    cv::SVD::compute(essential, D, U, Vt);

    float data[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    cv::Mat W(3, 3, CV_32FC1, data);

    if (cv::determinant(U) < 0) {
        U *= -1.0;
    }
    if (cv::determinant(Vt) < 0) {
        Vt *= -1.0;
    }
    R = U * W * Vt;
    U.col(2).copyTo(t);
    t /= cv::norm(t);
}

void normalize(const cv::Mat& K_i, const std::vector<cv::Point2f>& points, cv::Mat& normalized) {
    /* cv::Mat pts_h(points); */
    cv::Mat pts_h = cv::Mat::ones(3, points.size(), CV_32FC1);
    for (size_t i = 0; i < points.size(); i++) {
        pts_h.col(i).at<float>(0) = points[i].x;
        pts_h.col(i).at<float>(1) = points[i].y;
    }
    normalized = K_i * pts_h;
}

void normalize(const std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& normalized_points, cv::Mat& T) {
    float meanX = 0;
    float meanY = 0;
    const int N = keypoints.size();

    normalized_points.resize(N);

    for (int i = 0; i < N; i++) {
        meanX += keypoints[i].pt.x;
        meanX += keypoints[i].pt.y;
    }

    meanX /= N;
    meanY /= N;

    float meanStdDevX = 0;
    float meanStdDevY = 0;

    for (int i = 0; i < N; i++) {
        normalized_points[i].x = keypoints[i].pt.x - meanX;
        normalized_points[i].y = keypoints[i].pt.y - meanY;

        meanStdDevX += abs(normalized_points[i].x);
        meanStdDevY += abs(normalized_points[i].y);
    }

    meanStdDevX /= N;
    meanStdDevY /= N;

    float sigmaX = 1.0/meanStdDevX;
    float sigmaY = 1.0/meanStdDevY;

    for (int i = 0; i < N; i++) {
        normalized_points[i].x *= sigmaX;
        normalized_points[i].y *= sigmaY;
    }

    T = cv::Mat::eye(3, 3, CV_32FC1);
    T.at<float>(0,0) = sigmaX;
    T.at<float>(1,1) = sigmaY;
    T.at<float>(0,2) = -meanX*sigmaX;
    T.at<float>(1,2) = -meanY*sigmaY;
}

/* void findFundamental(const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::KeyPoint>& keypoints2, std::vector<bool> inliers, cv::Mat& fundamental) { */

/*     std::vector<cv::Point2f> k_n1, k_n2; */
/*     cv::Mat T1, T2; */
/*     normalize(keypoints1, k_n1, T1); */
/*     normalize(keypoints2, k_n2, T2); */
/*     cv::Mat T2_t = T2.t(); */

/*     float score = 0; */
/*     std::vector<cv::Point2f> k_n1i(8), k_n12(8); */
/*     cv::Mat F; */

/*     for (int i = 0; i < MAX_ITERATIONS; i++) { */
/*         for (int j = 0; j < 8; j++) { */
/*             int index = */


/* } */

cv::Mat computeFundamental(const std::vector<cv::Point2f> &p1, const std::vector<cv::Point2f> &p2) {
    const int N = p1.size();
    cv::Mat L(N, 9, CV_32FC1);

    for (int i = 0; i < N; i++) {
        const float u1 = p1[i].x;
        const float v1 = p1[i].y;
        const float u2 = p2[i].x;
        const float v2 = p2[i].y;

        L.at<float>(i,0) = u2*u1;
        L.at<float>(i,1) = u2*v1;
        L.at<float>(i,2) = u2;
        L.at<float>(i,3) = v2*u1;
        L.at<float>(i,4) = v2*v1;
        L.at<float>(i,5) = v2;
        L.at<float>(i,6) = u1;
        L.at<float>(i,7) = v1;
        L.at<float>(i,8) = 1;
    }

    cv::Mat U, D, V_t;
    cv::SVDecomp(L, D, U, V_t, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    cv::Mat F_p = V_t.row(8).reshape(0, 3);
    cv::SVDecomp(F_p, D, U, V_t, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    //Enforce rank constraint
    D.at<float>(2,2) = 0;

    return U * cv::Mat::diag(D) * V_t;
}
