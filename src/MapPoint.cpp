#include "MapPoint.h"

MapPoint::MapPoint(Map *pointMap, const cv::Point2f& point, const cv::Scalar& color, int id) : point(point), color(color) {
    this->id = (id != -1) ? id : pointMap->add_point(*this);
}

cv::Vec3f MapPoint::homogeneous() {
    return cv::Vec3f(point.x, point.y, 1);
}

void MapPoint::remove() {
    //Frames length and index length **should** be the same
    for (size_t i = 0; i < frames.size(); i++) {
        frames[i]->mapPoints[indexes[i]] = NULL;
    }
    //TODO: might be wrong
    delete this;
}

void MapPoint::add_observation(Frame& frame, int index) {
    if (frame.mapPoints[index] == NULL && std::find(frames.begin(), frames.end(), &frame) != frames.end()) {
        frame.mapPoints[index] = this;
        frames.push_back(&frame);
        indexes.push_back(index);
    }
}
