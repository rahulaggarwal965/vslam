#include "Map.h"
#include <memory>

int Map::add_point(std::shared_ptr<MapPoint> point) {
    this->mapPoints.push_back(point);
    return this->maxPoint++;
}

int Map::add_frame(std::shared_ptr<Frame> frame) {
    this->frames.push_back(frame);
    return this->maxFrame++;
}
