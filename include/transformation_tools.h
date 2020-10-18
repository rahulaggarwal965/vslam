#ifndef transformation_tools_h
#define transformation_tools_h

#include <opencv2/core.hpp>

void triangulate(const cv::Mat &pose1, const cv::Mat& pose2, const cv::Point2f &p1, const cv::Point2f &p2, cv::Mat& r3d);
void poseRt(const cv::Mat &R, const cv::Mat &t, cv::Mat &pose);
void fundamentalMatrixToRt(const cv::Mat& fundamental_matrix, const cv::Mat& K, cv::Mat& R, cv::Mat& t);
void normalize(const cv::Mat& K_i, const std::vector<cv::Point2f>& points, cv::Mat& normalized);
void normalize(const std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& normalized_points, cv::Mat& T);
cv::Mat computeFundamental(const std::vector<cv::Point2f> &p1, const std::vector<cv::Point2f> &p2);

#endif
