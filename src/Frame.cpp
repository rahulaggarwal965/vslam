#include "Frame.h"

Frame::Frame(const cv::Mat& image, const cv::Mat& K, const cv::Mat pose) :
K(K), pose(pose) {

    if (!image.empty()) {
    }


}
