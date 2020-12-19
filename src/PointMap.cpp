#include <PointMap.h>

void add_points(PointMap &pm, const cv::Mat &new_points) {
    int num_new_points = new_points.rows;
    if (pm.size + num_new_points > pm.capacity) {
        // Resize
        // TODO(rahul): examine this 2 here, how often are we reallocating
        int new_capacity = pm.capacity * 2 + num_new_points;

        // TODO(rahul): N x 4?
        cv::Mat points(new_capacity, 3, CV_32F);
        pm.capacity = new_capacity;

        pm.points.copyTo(points.rowRange(0, pm.size));
        new_points.copyTo(pm.points.rowRange(pm.size, pm.capacity));
        pm.size = pm.size + num_new_points;

        pm.points = points;
    }
}

void add_reprojection_inliers(PointMap &pm, const cv::Mat &points_4d, const std::vector<memory_index> &reprojection_inliers) {
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

    f32 *points_data = pm.points.ptr<float>(pm.size);
    const f32 *p4d_data = points_4d.ptr<float>();
    memory_index j = 0;
    for (memory_index row : reprojection_inliers) {
        const memory_index i = row * 4;
        points_data[j]     = p4d_data[i];
        points_data[j + 1] = p4d_data[i + 1];
        points_data[j + 2] = p4d_data[i + 2];
        points_data[j + 3] = p4d_data[i + 3];
        j += 4;
    }
    pm.size += num_new_points;
}
