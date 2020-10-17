#ifndef MapPoint_h
#define MapPoint_h

#include "opencv2/core/types.hpp"
#include "opencv2/core/base.hpp"
#include <algorithm>
#include <memory>
#include "Map.h"
#include "Frame.h"

class MapPoint {

public:
    cv::Vec3f point;

    std::vector<std::shared_ptr<Frame>> frames;     //Frames that this point is in
    std::vector<int> indexes;       //Indexes of matches
    cv::Scalar color;
    int id;

    MapPoint(Map& pointMap, const cv::Vec3f& point, const cv::Scalar& color, int id = -1);

    cv::Vec3f homogeneous();
    void remove();
    double orb_distance(const cv::Mat& descriptor);
    void add_observation(std::shared_ptr<Frame> frame, int index);


};

#endif
