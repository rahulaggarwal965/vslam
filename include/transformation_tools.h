#ifndef transformation_tools_h
#define transformation_tools_h

#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/types.hpp"
#include <iostream>
#include <iterator>
#include <opencv2/core.hpp>
#include <random>

void triangulate(const cv::Mat &pose1, const cv::Mat &pose2,
                 const cv::Point2f &p1, const cv::Point2f &p2, cv::Mat &r3d);
void poseRt(const cv::Mat &R, const cv::Mat &t, cv::Mat &pose);
void fundamentalMatrixToRt(const cv::Mat &fundamental_matrix, const cv::Mat &K,
                           cv::Mat &R, cv::Mat &t);
void normalize(const std::vector<cv::KeyPoint> &keypoints,
               std::vector<cv::Point2f> &normalized_points, cv::Mat &T);
void findFundamental(const std::vector<cv::KeyPoint> &keypoints1,
                     const std::vector<cv::KeyPoint> &keypoints2,
                     std::vector<bool> &inliers, cv::Mat &fundamental);
cv::Mat computeFundamental(const std::vector<cv::Point2f> &p1,
                           const std::vector<cv::Point2f> &p2);
float fundamentalError(const std::vector<cv::KeyPoint> &keypoints1,
                       const std::vector<cv::KeyPoint> &keypoints2,
                       const cv::Mat &F, std::vector<bool> &inliers,
                       int &nInliers, float thresh);

#endif
