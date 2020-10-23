#ifndef __RASNAC_FILTER_H__
#define __RANSAC_FILTER_H__

#include "opencv2/core/types.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core.hpp"
#include <cstdlib>
#include <random>
#include <vector>

class RansacFilter {

    public:
        const int min_items;
        const int max_iterations;
        const float threshold;

        RansacFilter(const int min_items = 8, const int max_iterations = 100, const float threshold = 0.2);

        void initialize(std::vector<cv::KeyPoint> *kp1, std::vector<cv::KeyPoint> *kp2, std::vector<std::pair<int, int>> *matches);
        void find_fundamental(std::vector<bool> *inliers, cv::Mat *fundamental);
        void compute_fundamental(std::vector<cv::Point2f> *kp1_set, std::vector<cv::Point2f> *kp2_set, cv::Mat *temp_F);
        std::pair<int, float> compute_fundamental_residual(cv::Mat *F, std::vector<bool> *inliers);

    private:
        std::vector<std::vector<int>> ransac_sets;
        std::vector<cv::KeyPoint> *kp1, *kp2;
        std::vector<std::pair<int, int>> *matches;

};

#endif
