#include "Frame.h"
#include "opencv2/flann/miniflann.hpp"

Frame::Frame(const cv::Mat& image, const cv::Mat& K, const cv::Mat pose) :
K(K), pose(pose) {
    cv::invert(K, K_inv);
    if (!image.empty()) {
        extract_key_points(image, keypoints, descriptors);
        //TODO: NORMALIZE KEY POINTS

        mapPoints.resize(keypoints.size(), NULL);
    }
}

void Frame::generate_kdtree() {
    cv::Mat_<float> features(0, 2);
    for (auto& point : keypoints) {
        cv::Mat row = (cv::Mat_<float>(1, 2) << point.pt.x, point.pt.y);
        features.push_back(row);
    }
    kdtree = cv::flann::Index(features, cv::flann::KDTreeIndexParams(1));
}
