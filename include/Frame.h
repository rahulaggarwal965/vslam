#ifndef __FRAME_H__
#define __FRAME_H__

#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/features2d.hpp>
#include "helpers.h"
#include "RansacFilter.h"
#include "KDTree.h"

struct Frame {
    cv::Mat image;
#if 0
    float pose[16], R_t[16];
#else
    cv::Mat pose, R_t; // really should be 16 + 16 = 32 floats * 4 = 128 bytes
#endif

    /* std::vector<cv::KeyPoint> keypoints; */
    std::vector<cv::Point2f> points;
    std::vector<s32> map_point_ids;
    cv::Mat descriptors;

    frame_kdtree kdtree;

    u64 id;
};

void initialize_frame(Frame &frame, const cv::Mat &image, long frame_id);
void draw(const Frame &frame, cv::Mat &annotated);
void generate_kdtree(const Frame &frame);
void extract_features(Frame &frame, int nrows, int ncols);
void extract_features(Frame &frame);
void match_features(const Frame &frame1, const Frame &frame2, RansacFilter &rf, std::vector<std::pair<int, int>> &matches, cv::Mat &F);

#endif
