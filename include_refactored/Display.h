#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include "Frame.h"
#include <opencv2/core.hpp>
#include <pangolin.h>
#include <thread>
#include <queue>

struct DisplayState {
    std::vector<cv::Point3f> *points = NULL;
    std::vector<Frame> *frames = NULL;
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
        void draw_box(const cv::Mat &pose, float w = 1.0, float h_ratio = 0.75, float z_ratio = 0.6);
        void draw_boxes(std::vector<cv::Mat*> boxes, float w=1.0, float h_ratio=0.75, float z_ratio=0.6);

    private:
        std::thread loop;
        std::mutex *mtx;
        const char *window_name;
        const int W, H;
};

#endif
