#ifndef frame_h
#define frame_h

#include "opencv2/core.hpp"
#include "opencv2/core/mat.hpp"

//TODO: this is really fucking ugly
void extract_key_points(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

class Frame {

public:
  // Camera Intrinsic
  cv::Mat image, K, K_inv, pose;
  std::vector<cv::KeyPoint> keypoints, normalized_keypoints;
  cv::Mat descriptors;
  int id;

  Frame(const cv::Mat &image, const cv::Mat &K,
        const cv::Mat pose = cv::Mat::eye(4, 4, CV_32FC1));
};

#endif
