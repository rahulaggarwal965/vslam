#ifndef __RASNAC_FILTER_H__
#define __RANSAC_FILTER_H__

#include "opencv2/core/types.hpp"
#include <cstdlib>
#include <random>
#include <vector>

class RansacFilter {

    public:
        const int min_items = 8;
        const int max_iterations = 100;
        const float threshold = 0.02;

        void findFundmental(std::vector<cv::KeyPoint> *kp1, std::vector<cv::KeyPoint> *kp2, std::vector<std::pair<int, int>> *matches, std::vector<bool> *inliers, cv::Mat *fundamental) {

            const int N = matches->size();

            //creating sets
            std::vector<int> all_indices, available_indices;
            all_indices.reserve(N);
            for (int i = 0; i < N; i++) {
                all_indices.push_back(i);
            }

            ransac_sets = std::vector<std::vector<int>>(max_iterations, std::vector<int>(8, 0));

            for (int i = 0; i < max_iterations; i++) {
                available_indices = all_indices;
                std::random_device rd;
                std::mt19937 gen(rd());

                for (int j = 0; j < min_items; j++) {
                    std::uniform_int_distribution<> distr(0, available_indices.size() - 1);

                    int rand = distr(gen);

                    ransac_sets[i][j] = available_indices[rand];

                    available_indices[rand] = available_indices.back();
                    available_indices.pop_back();
                }
            }
        }
    private:
        std::vector<std::vector<int>> ransac_sets;
        std::vector<cv::KeyPoint> kp1, kp2;
        std::vector<std::pair<int, int>> matches;

}

#endif
