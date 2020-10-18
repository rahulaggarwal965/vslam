#include "MapPoint.h"
#include "Frame.h"

MapPoint::MapPoint(const cv::Vec3f& point, const cv::Scalar& color, int id) : point(point), color(color) {}

cv::Vec3f MapPoint::homogeneous() {
    return cv::Vec3f(point[0], point[1], 1);
}

void MapPoint::remove() {
    //Frames length and index length **should** be the same
    for (size_t i = 0; i < frames.size(); i++) {
        frames[i]->mapPoints[indexes[i]].lock() = NULL;
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

void MapPoint::add_observation(std::shared_ptr<Frame> frame, std::shared_ptr<MapPoint> point, int index) {
    if (frame->mapPoints[index].lock() == NULL && std::find(frames.begin(), frames.end(), frame) != frames.end()) {
        frame->mapPoints[index] = point;
        frames.push_back(frame);
        indexes.push_back(index);
    }
}
