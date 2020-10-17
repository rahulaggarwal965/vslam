#ifndef Map_h
#define Map_h

#include <vector>

class Frame;
class MapPoint;

class Map {

public:
    std::vector<Frame*> frames;
    std::vector<MapPoint*> mapPoints;
    int maxFrame = 0, maxPoint = 0;

    //TODO: to implement last
    /* void serialize(char** filepath); */
    /* void deserialize(char** filepath); */

    int add_point(MapPoint& mapPoint);
    int add_frame(Frame& frame);

};

#endif
