#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <opencv2/core.hpp>
#include <pangolin.h>
#include <thread>
#include <queue>

#include "vslam_internal.h"
#include "Frame.h"

struct DisplayState {
    cv::Mat *points = NULL;
    usize size = 0;
    std::vector<Frame> *frames = NULL;
    std::vector<cv::Point3_<u8>> *colors = NULL;
};

class Display {

    public:
        Display(const char *window_name, const int W, const int H, std::mutex *mtx);
        /* std::queue<DisplayState> q; */
        DisplayState ds;

        void initialize();
        void run();
        void close();
        void join();

        void draw_points(const std::vector<cv::Point3f> &points);
        void draw_points(const cv::Mat &points, const usize size);
        void draw_points_colors(const cv::Mat &points, const std::vector<cv::Point3_<u8>> &colors, const usize size);
        void draw_box(const cv::Mat &pose, float w = 1.0, float h_ratio = 0.75, float z_ratio = 0.6);
        void draw_boxes(std::vector<cv::Mat*> boxes, float w=1.0, float h_ratio=0.75, float z_ratio=0.6);

    private:
        std::thread loop;
        std::mutex *mtx;
        const char *window_name;
        const int W, H;
};

#endif
