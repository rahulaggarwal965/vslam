#ifndef __POINT_MAP_H__
#define __POINT_MAP_H__

#include <opencv2/core.hpp>
#include <vector>

#include "Frame.h"
#include "vslam_internal.h"

struct PointMap {
    memory_index size = 0;
    memory_index capacity = 0;

    cv::Mat points;
    std::vector<std::vector<memory_index>> frame_ids;
    std::vector<std::vector<memory_index>> frame_point_ids;


    std::vector<Frame> frames;
};

void add_points(PointMap &pm, const cv::Mat &new_points);
void add_reprojection_inliers(PointMap &pm, const cv::Mat &points_4d, const std::vector<memory_index> &reprojection_inliers);
u32 orb_distance(const PointMap &pm, memory_index map_point_id, const Frame &frame, memory_index frame_point_id);

#endif
