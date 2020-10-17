#ifndef feature_matching_h
#define feature_matching_h

#include "opencv2/core/types.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv4/opencv2/core/mat.hpp>
#include "Frame.h"
#include <set>

void extract_key_points(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

void match_frames(const Frame& frame1, const Frame& frame2, const cv::Mat& K, std::vector<int>& idx1, std::vector<int>& idx2, cv::Mat& fundamental_matrix);

#endif
