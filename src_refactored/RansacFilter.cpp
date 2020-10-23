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

    std::random_device rd;
    std::mt19937 gen(rd());
    ransac_sets = std::vector<std::vector<int>>(max_iterations, std::vector<int>(8, 0));

    for (int i = 0; i < max_iterations; i++) {
        available_indices = all_indices;

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
    float best_score = 0;
    int best_nInliers = 0;

    std::vector<bool> current_inliers;

    for (int i = 0; i < max_iterations; i++) {
        for (size_t j = 0; j < ransac_sets[i].size(); j++) {
            int idx = ransac_sets[i][j];
            kp1_set[j] = (*kp1)[(*matches)[idx].first].pt;
            kp2_set[j] = (*kp2)[(*matches)[idx].second].pt;
        }

        compute_fundamental(&kp1_set, &kp2_set, &temp_F);
        auto r = compute_fundamental_residual(&temp_F, &current_inliers); // nInliers, sum of residuals

        if (r.first > best_nInliers || (r.first == best_nInliers && r.second > best_score)) {
            /* printf("Number of inliers: %d, Sum of squared residuals: %f\n", r.first, r.second); */
            best_nInliers = r.first;
            best_score = r.second;
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

std::pair<int, float> RansacFilter::compute_fundamental_residual(cv::Mat *F, std::vector<bool> *inliers) {
    const int N = matches->size();
    inliers->resize(N);
    cv::Mat x1(3, N, CV_32FC1), x2(3, N, CV_32FC1);

    for (int i = 0; i < N; i++) {
        x1.at<float>(0, i) = (*kp1)[(*matches)[i].first].pt.x;
        x1.at<float>(1, i) = (*kp1)[(*matches)[i].first].pt.y;
        x1.at<float>(2, i) = 1;
        x2.at<float>(0, i) = (*kp2)[(*matches)[i].second].pt.x;
        x2.at<float>(1, i) = (*kp2)[(*matches)[i].second].pt.y;
        x2.at<float>(2, i) = 1;
    }

    cv::Mat F_x1 = (*F) * x1;       // 3xN
    cv::Mat F_t_x2 = (*F).t() * x2; // 3xN

    cv::Mat x2_t_F_x1 = x2.mul(F_x1); // 3xN -> 1xN
    cv::reduce(x2_t_F_x1, x2_t_F_x1, 0, cv::REDUCE_SUM);

    //TODO: cleanup
    cv::Mat e_sq = x2_t_F_x1.mul(x2_t_F_x1) / F_x1.row(0).mul(F_x1.row(0)) + F_x1.row(1).mul(F_x1.row(1)) + F_t_x2.row(0).mul(F_t_x2.row(0)) + F_t_x2.row(1).mul(F_t_x2.row(1)); // 1xN

    int nInliers = 0;
    for (int i = 0; i < N; i++) {
        if (e_sq.at<float>(i) <= threshold) {
            (*inliers)[i] = true;
            nInliers++;
        } else {
            (*inliers)[i] = false;
        }
    }

    return std::make_pair(nInliers, (float) cv::sum(e_sq)[0]); // 1

}
