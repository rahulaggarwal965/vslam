#ifndef frame_h
#define frame_h

#include "opencv2/core/mat.hpp"

class Frame {

public:
    //Camera Intrinsic
    cv::Mat image, K, pose;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    Frame(const cv::Mat& image, const cv::Mat& K, const cv::Mat pose = cv::Mat::eye(4, 4, CV_32FC1));

};

#endif
