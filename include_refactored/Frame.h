#ifndef __FRAME_H__
#define __FRAME_H__

#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/features2d.hpp>
#include "helpers.h"
#include "RansacFilter.h"

struct Frame {
    cv::Mat image, pose, R_t;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    long id;
};

void initialize_frame(Frame &frame, const cv::Mat &image);
void draw(const Frame &frame, cv::Mat &annotated);
void generate_kdtree(const Frame &frame);
void extract_features(Frame &frame, int nrows, int ncols);
void extract_features(Frame &frame);
void match_features(const Frame &frame1, const Frame &frame2, RansacFilter &rf, std::vector<std::pair<int, int>> &matches, cv::Mat &F);

#endif
