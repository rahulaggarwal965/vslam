#include <PointMap.h>

void add_reprojection_inliers(PointMap &pm, const cv::Mat &points_4d, const std::vector<memory_index> &reprojection_inliers, const std::vector<cv::Point3_<u8>> &colors, u64 last_frame_id, u64 frame_id, const std::vector<std::pair<int, int>> &matches) {
    const memory_index num_new_points = reprojection_inliers.size();
    if (pm.size + num_new_points > pm.capacity) {
        memory_index new_capacity = pm.capacity * 2 + num_new_points;

        cv::Mat points(new_capacity, 4, CV_32F);
        pm.capacity = new_capacity;

        if (pm.size != 0) {
            pm.points.rowRange(0, pm.size).copyTo(points.rowRange(0, pm.size));
        }
        pm.points = points;
    }

    pm.frame_point_ids.reserve(num_new_points);
    pm.colors.insert(pm.colors.end(), colors.begin(), colors.end());
    f32 *points_data = pm.points.ptr<float>(pm.size);
    const f32 *p4d_data = points_4d.ptr<float>();
    memory_index j = 0;
    for (memory_index row : reprojection_inliers) {
        const memory_index i = row * 4;
        points_data[j]     = p4d_data[i];
        points_data[j + 1] = p4d_data[i + 1];
        points_data[j + 2] = p4d_data[i + 2];
        points_data[j + 3] = 1;
        j += 4;
        pm.frame_point_ids.push_back({static_cast<memory_index>(matches[row].first), static_cast<memory_index>(matches[row].second)});
    }
    pm.size += num_new_points;
    pm.frame_ids.resize(pm.size, {last_frame_id, frame_id});
    printf("n_colors: %zu\n", pm.colors.size());
}

u32 orb_distance(const PointMap &pm, memory_index map_point_id, const Frame &frame, memory_index frame_point_id) {
    u32 min = u32_max;
    const std::vector<memory_index> &frame_ids = pm.frame_ids[map_point_id];
    const std::vector<memory_index> &frame_point_ids = pm.frame_point_ids[map_point_id];
    // NOTE(rahul): size of both vectors should be the same
    for (memory_index i = 0; i < frame_ids.size(); i++) {
        u32 curr = cv::norm(frame.descriptors.row(frame_point_id), pm.frames[frame_ids[i]].descriptors.row(frame_point_ids[i]), cv::NORM_HAMMING);
        if (curr < min) min = curr;
    }
    return min;
}
