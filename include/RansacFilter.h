#ifndef __RANSAC_FILTER_H__
#define __RANSAC_FILTER_H__

#include <opencv2/core.hpp>
#include <cstdlib>
#include <random>
#include <vector>

class RansacFilter {

    public:
        const int min_items;
        const int max_iterations;
        const float threshold;

        RansacFilter(const int min_items = 8, const int max_iterations = 100, const float threshold = 0.2);

        void initialize_sets(const int n_matches);
        void find_fundamental(const std::vector<cv::Point2f> &p1, const std::vector<cv::Point2f> &p2, const std::vector<std::pair<int, int>> &matches, std::vector<bool> &inliers, cv::Mat &fundamental);
        void compute_fundamental(const std::vector<cv::Point2f> &p1_set, const std::vector<cv::Point2f> &p2_set, cv::Mat &temp_F);
        std::pair<int, float> compute_fundamental_residual(const std::vector<cv::Point2f> &p1, const std::vector<cv::Point2f> &p2, const std::vector<std::pair<int, int>> &matches, const cv::Mat &F, std::vector<bool> &inliers);

    private:
        std::vector<std::vector<int>> ransac_sets;
};

#endif
