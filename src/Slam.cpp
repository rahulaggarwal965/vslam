#include "Slam.h"

Slam::Slam(int W, int H, const cv::Mat& K) : point_map(), K(K), W(W), H(H) {}

void Slam::process_frame(const cv::Mat &image) {
    Frame frame(image, K);
    frame.id = point_map->add_frame(frame);

    if (frame.id == 0) return;

    size_t num_frames = point_map->frames.size();
    Frame *frame1 = point_map->frames[num_frames - 1];
    Frame *frame2 = point_map->frames[num_frames - 2];

    std::vector<int> idx1, idx2;
    cv::Mat pose, Rt_i;

    match_frames(*frame1, *frame2, K, idx1, idx2, pose);

    for (size_t i = 0; i < idx2.size(); i++) {
        if (frame2->mapPoints[idx2[i]] != NULL && frame1->mapPoints[idx1[i]] == NULL) {
            frame2->mapPoints[idx2[i]]->add_observation(*frame1, idx1[i]);
        }
    }


    //TODO: switch so no inversion?
    cv::invert(pose, Rt_i);
    /* if (frame.id < 5) { */
    frame1->pose = Rt_i * frame2->pose;
    /* } else { */
        //TODO: Kinematic Model, constant velocity
    /* } */

    int projection_point_count = 0;

    if (int s = point_map->mapPoints.size() > 0) {
        //TODO: refactor to matrix operations (don't use homogeneous)
        cv::Mat mapPoints(3, s, CV_32FC1);
        for (int i = 0; i < s; i++) {
            mapPoints.col(i) = point_map->mapPoints[i]->homogeneous();
        }
        cv::Mat projections = (K * frame1->pose(cv::Range(0, 3), cv::Range(0, 3))) * mapPoints;
        std::vector<cv::Vec2f> projectionPoints;
        std::vector<bool> goodPoints;

        projectionPoints.reserve(s);
        goodPoints.reserve(s);
        for (int i = 0; i < s; i++) {
            cv::Vec3f h = mapPoints.col(i);
            h /= h[2];
            projectionPoints.emplace_back(h[0], h[1]);
            if (h[0] > 0 && h[0] < W && h[1] > 0 && h[1] < H) {
                goodPoints.push_back(true);
            } else {
                goodPoints.push_back(false);
            }
        }
        for (int i = 0; i < s; i++) {
            if (!goodPoints[i]) continue;
            if (std::find(point_map->mapPoints[i]->frames.begin(), point_map->mapPoints[i]->frames.end(), frame1) != point_map->mapPoints[i]->frames.end()) {
                continue;
            }
        }
    }

}
