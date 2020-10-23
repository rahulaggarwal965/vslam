#include "RansacFilter.h"

RansacFilter::RansacFilter(const int min_items, const int max_iterations, const float threshold) :
    min_items(min_items), max_iterations(max_iterations), threshold(threshold) {}

void RansacFilter::initialize(std::vector<cv::KeyPoint> *kp1, std::vector<cv::KeyPoint> *kp2, std::vector<std::pair<int, int>> *matches) {
    const int N = matches->size();

    this->kp1 = kp1;
    this->kp2 = kp2;
    this->matches = matches;

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

void RansacFilter::find_fundamental(std::vector<bool> *inliers, cv::Mat *fundamental) {
    //TODO: normalize

    std::vector<cv::Point2f> kp1_set(8), kp2_set(8);
    cv::Mat temp_F;
    int best_score = 0, current_score = 0;
    int best_nInliers = 0;

    std::vector<bool> current_inliers;

    for (int i = 0; i < max_iterations; i++) {
        for (int j = 0; j < ransac_sets[i].size(); j++) {
            int idx = ransac_sets[i][j];
            kp1_set[j] = (*kp1)[(*matches)[idx].first].pt;
            kp2_set[j] = (*kp2)[(*matches)[idx].second].pt;
        }

        compute_fundamental(&kp1_set, &kp2_set, &temp_F);
        current_score = compute_fundamental_residual(&temp_F, &current_inliers);

        if (current_inliers.size() > best_nInliers || (current_inliers.size() == best_nInliers && current_score > best_score)) {
            best_nInliers = current_inliers.size();
            best_score = current_score;
            temp_F.copyTo(*(fundamental));
            inliers->swap(current_inliers);
        }
    }
}

void RansacFilter::compute_fundamental(std::vector<cv::Point2f> *kp1_set, std::vector<cv::Point2f> *kp2_set, cv::Mat *temp_F) {
    const int N = kp1_set->size();

    cv::Mat A(N, 9, CV_32FC1);

    for (int i = 0; i < N; i++) {
        const float u1 = (*kp1_set)[i].x;
        const float v1 = (*kp1_set)[i].y;
        const float u2 = (*kp2_set)[i].x;
        const float v2 = (*kp2_set)[i].y;

        // dst' * F * src = 0
        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1;
    }

    cv::Mat U, D, V_t;
    //solve dst' * F * src = 0 in the form of A * f = 0
    cv::SVDecomp(A, D, U, V_t, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    cv::Mat F = V_t.row(8).reshape(0, 3);

    //Enforce third eigenvalue constraint being 0 (rank 2)
    cv::SVDecomp(F, D, U, V_t, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    D.at<float>(2) = 0;

    *temp_F = U * cv::Mat::diag(D) * V_t;

}

int RansacFilter::compute_fundamental_residual(cv::Mat *temp_F, std::vector<bool> *inliers) {

}
