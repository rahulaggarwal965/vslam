#ifndef frame_h
#define frame_h

#include "Map.h"
#include "opencv2/core.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/flann/miniflann.hpp"
#include <memory>
#include <opencv2/ml.hpp>

class MapPoint;

//TODO: this is really fucking ugly
void extract_key_points(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

class Frame {

public:
  // Camera Intrinsic
  cv::Mat image, K, K_inv, pose;
  std::vector<cv::KeyPoint> keypoints, normalized_keypoints;
  std::vector<std::weak_ptr<MapPoint>> mapPoints;
  cv::flann::Index kdtree;
  cv::Mat descriptors;
  int id;

  Frame(const cv::Mat &image, const cv::Mat &K,
        const cv::Mat pose = cv::Mat::eye(4, 4, CV_32FC1));
  void draw(const cv::Mat& image, cv::Mat& drawn);
  void generate_kdtree();

};

#endif
