#include "Map.h"

int Map::add_point(MapPoint& mapPoint) {
    this->mapPoints.push_back(&mapPoint);
    return this->maxPoint++;
}

int Map::add_frame(std::shared_ptr<Frame> frame) {
    this->frames.push_back(frame);
    return this->maxFrame++;
}
