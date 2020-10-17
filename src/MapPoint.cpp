#include "MapPoint.h"
#include "opencv2/core/base.hpp"

MapPoint::MapPoint(Map *pointMap, const cv::Vec3f& point, const cv::Scalar& color, int id) : point(point), color(color) {
    this->id = (id != -1) ? id : pointMap->add_point(*this);
}

cv::Vec3f MapPoint::homogeneous() {
    return cv::Vec3f(point[0], point[1], 1);
}

void MapPoint::remove() {
    //Frames length and index length **should** be the same
    for (size_t i = 0; i < frames.size(); i++) {
        frames[i]->mapPoints[indexes[i]] = NULL;
    }
    //TODO: might be wrong
    delete this;
}

double MapPoint::orb_distance(const cv::Mat& descriptor) {
    double min = std::numeric_limits<double>::max();
    for (size_t i = 0; i < frames.size(); i++) {
        double temp = cv::norm(frames[i]->descriptors.row(indexes[i]), descriptor, cv::NORM_HAMMING);
        if (temp < min) min = temp;
    }
    return min;
}

void MapPoint::add_observation(Frame& frame, int index) {
    if (frame.mapPoints[index] == NULL && std::find(frames.begin(), frames.end(), &frame) != frames.end()) {
        frame.mapPoints[index] = this;
        frames.push_back(&frame);
        indexes.push_back(index);
    }
}
