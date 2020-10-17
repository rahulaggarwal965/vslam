#ifndef MapPoint_h
#define MapPoint_h

#include "opencv2/core/types.hpp"
#include "Map.h"

class Frame;

class MapPoint {

public:
    cv::Point2f point;

    std::vector<Frame*> frames;     //Frames that this point is in
    std::vector<int> indexes;       //Indexes of matches
    cv::Scalar color;
    int id;

    MapPoint(Map* pointMap, const cv::Point2f& point, const cv::Scalar& color, int id = -1);

    cv::Point3f homogeneous();


};

#endif
