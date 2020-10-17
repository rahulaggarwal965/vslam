#include "Slam.h"

Slam::Slam(const cv::Mat& K) : point_map(), K(K) {}

void Slam::process_frame(const cv::Mat &image) {
    Frame frame(image, K);
    frame.id = point_map->add_frame(frame);

    if (frame.id == 0) return;

    size_t num_frames = point_map->frames.size();
    Frame* frame1 = point_map->frames[num_frames - 1];
    Frame* frame2 = point_map->frames[num_frames - 2];
}
