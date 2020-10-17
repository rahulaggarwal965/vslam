#ifndef slam_h
#define slam_h

#include "Frame.h"
#include "Map.h"
#include "MapPoint.h"
#include "opencv2/core/mat.hpp"
#include "feature_matching.h"

class Slam {

public:
    Map point_map;
    cv::Mat K;
    int W, H;

    Slam(int W, int H, const cv::Mat& K);
    void process_frame(const cv::Mat& image);

};

#endif
