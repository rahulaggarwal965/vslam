#include "optimizer.h"
#include <vector>

struct optimizer {
    std::vector<cv::Mat> initial_poses;
    std::vector<cv::Mat> landmark_priors;
    std::vector<cv::Point3f> measurements;
}

