#include "MapPoint.h"

MapPoint::MapPoint(Map *pointMap, const cv::Point2f& point, const cv::Scalar& color, int id) : point(point), color(color) {
    this->id = (id != -1) ? id : pointMap->add_point(*this);
}

cv::Point3f MapPoint::homogeneous() {
    return cv::Point3f(point.x, point.y, 1);
}



