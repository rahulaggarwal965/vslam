#ifndef slam_h
#define slam_h

#include "Frame.h"
#include "Map.h"
#include "opencv2/core/mat.hpp"

class Slam {

public:
    Map *point_map;
    cv::Mat K;

    Slam(const cv::Mat& K);
    void process_frame(const cv::Mat& image);

};

#endif
