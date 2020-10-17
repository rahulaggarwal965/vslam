#include "Frame.h"

Frame::Frame(const cv::Mat& image, const cv::Mat& K, const cv::Mat pose) :
K(K), pose(pose) {
    cv::invert(K, K_inv);
    if (!image.empty()) {
        extract_key_points(image, keypoints, descriptors);
        //TODO: NORMALIZE KEY POINTS

        mapPoints.resize(keypoints.size(), NULL);
    }
}
