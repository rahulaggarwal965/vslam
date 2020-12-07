#ifndef __HELPER_H__
#define __HELPER_H__

#include <opencv2/core.hpp>
#include <iostream>

inline void print_matrix(const cv::Mat &mat, const char *name) {
    std::cout << name << '\n' << mat << '\n' << '\n';
}

inline void write_matrix(const cv::Mat &mat, const char *name, cv::FileStorage &fs) {
    fs << name << mat;
}

void extract_Rt(const cv::Mat &fundamental, const cv::Mat &K, cv::Mat &rotation, cv::Mat &translation);

void triangulate(const cv::Mat &p1, const cv::Mat &p2, const cv::Mat &c1, const cv::Mat &c2, cv::Mat &points_4d);

#endif
