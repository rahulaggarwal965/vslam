#include "transformation_tools.h"

#define MAX_ITERATIONS 200

void triangulate(const cv::Mat &pose1, const cv::Mat& pose2, const cv::Point2f &p1, const cv::Point2f &p2, cv::Mat& r3d) {
    cv::Mat E(4, 4, CV_32FC1);

    E.row(0) = p1.x*pose1.row(2)-pose1.row(0);
    E.row(1) = p1.y*pose1.row(2)-pose1.row(1);
    E.row(2) = p2.x*pose2.row(2)-pose2.row(0);
    E.row(3) = p2.y*pose2.row(2)-pose2.row(1);

    cv::Mat U, D, V_t;
    cv::SVDecomp(E, D, U, V_t, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    r3d = V_t.row(3).t();
    r3d = r3d.rowRange(0, 3)/r3d.at<float>(3);

}

void poseRt(const cv::Mat &R, const cv::Mat &t, cv::Mat &pose) {
  pose = cv::Mat::eye(4, 4, CV_32F);
  R.copyTo(pose.rowRange(0, 3).colRange(0, 3));
  t.copyTo(pose.rowRange(0, 3).col(3));
}

void fundamentalMatrixToRt(const cv::Mat& fundamental_matrix, const cv::Mat& K, cv::Mat& R, cv::Mat& t) {
    std::cout << "Fundamental Matrix: " << "\n" << fundamental_matrix << "\n" << "\n";
    cv::Mat F;
    fundamental_matrix.convertTo(F, CV_32FC1);
    cv::Mat essential = K.t()* F * K;

    cv::Mat U, D, Vt;
    cv::SVD::compute(essential, D, U, Vt);

    float data[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    cv::Mat W(3, 3, CV_32FC1, data);

    if (cv::determinant(U) < 0) {
        U *= -1.0;
    }
    if (cv::determinant(Vt) < 0) {
        Vt *= -1.0;
    }
    R = U * W * Vt;
    U.col(2).copyTo(t);
    t /= cv::norm(t);
}


void normalize(const std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& normalized_points, cv::Mat& T) {
    float meanX = 0;
    float meanY = 0;
    const int N = keypoints.size();

    normalized_points.resize(N);

    for (int i = 0; i < N; i++) {
        meanX += keypoints[i].pt.x;
        meanX += keypoints[i].pt.y;
    }

    meanX /= N;
    meanY /= N;

    float meanStdDevX = 0;
    float meanStdDevY = 0;

    for (int i = 0; i < N; i++) {
        normalized_points[i].x = keypoints[i].pt.x - meanX;
        normalized_points[i].y = keypoints[i].pt.y - meanY;

        meanStdDevX += abs(normalized_points[i].x);
        meanStdDevY += abs(normalized_points[i].y);
    }

    meanStdDevX /= N;
    meanStdDevY /= N;

    float sigmaX = 1.0/meanStdDevX;
    float sigmaY = 1.0/meanStdDevY;

    for (int i = 0; i < N; i++) {
        normalized_points[i].x *= sigmaX;
        normalized_points[i].y *= sigmaY;
    }

    T = cv::Mat::eye(3, 3, CV_32FC1);
    T.at<float>(0,0) = sigmaX;
    T.at<float>(1,1) = sigmaY;
    T.at<float>(0,2) = -meanX*sigmaX;
    T.at<float>(1,2) = -meanY*sigmaY;
}

void findFundamental(const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::KeyPoint>& keypoints2, std::vector<bool>& inliers, cv::Mat& fundamental) {

    const int N = keypoints1.size();

    std::vector<size_t> all_idxs;
    all_idxs.reserve(N);
    std::vector<size_t> avail_idxs;

    for (int i = 0; i < N; i++) {
        all_idxs.push_back(i);
    }

    std::vector<std::vector<size_t>> ransac_sets(MAX_ITERATIONS, std::vector<size_t>(8,0));

    for (int i = 0; i < MAX_ITERATIONS; i++) {

        avail_idxs = all_idxs;

        for (int j = 0; j < 8; j++) {

            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> distr(0, avail_idxs.size() - 1);

            int rand = distr(gen);
            int index = avail_idxs[rand];

            ransac_sets[i][j] = index;

            avail_idxs[rand] = avail_idxs.back();
            avail_idxs.pop_back();
        }
    }

    std::vector<cv::Point2f> k_n1, k_n2;
    cv::Mat T1, T2;
    normalize(keypoints1, k_n1, T1);
    normalize(keypoints2, k_n2, T2);
    cv::Mat T2_t = T2.t();

    int sum_resid = 0, best_nInliers = 0;
    inliers = std::vector<bool>(N, false);
    std::vector<bool> current_inliers(N, false);
    std::vector<cv::Point2f> k_n1i(8), k_n2i(8);
    cv::Mat F;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
        for (int j = 0; j < 8; j++) {
            int index = ransac_sets[i][j];

            k_n1i[j] = k_n1[index];
            k_n2i[j] = k_n2[index];
        }
        cv::Mat F_m = computeFundamental(k_n1i, k_n2i);
        F = T2_t*F_m*T1;

        //CHECK
        int curr_nInliers;
        int curr_sum_resid = fundamentalError(keypoints1, keypoints2, F, current_inliers, curr_nInliers, 0.02);
        if (curr_nInliers > best_nInliers || (curr_nInliers == best_nInliers && curr_sum_resid < sum_resid)) {
            fundamental = F.clone();
            inliers = current_inliers;
            best_nInliers = curr_nInliers;
            sum_resid = curr_sum_resid;
        }
    }


}

cv::Mat computeFundamental(const std::vector<cv::Point2f> &p1, const std::vector<cv::Point2f> &p2) {
    const int N = p1.size();
    cv::Mat L(N, 9, CV_32FC1);

    for (int i = 0; i < N; i++) {
        const float u1 = p1[i].x;
        const float v1 = p1[i].y;
        const float u2 = p2[i].x;
        const float v2 = p2[i].y;

        L.at<float>(i,0) = u2*u1;
        L.at<float>(i,1) = u2*v1;
        L.at<float>(i,2) = u2;
        L.at<float>(i,3) = v2*u1;
        L.at<float>(i,4) = v2*v1;
        L.at<float>(i,5) = v2;
        L.at<float>(i,6) = u1;
        L.at<float>(i,7) = v1;
        L.at<float>(i,8) = 1;
    }

    cv::Mat U, D, V_t;
    cv::SVDecomp(L, D, U, V_t, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    cv::Mat F_p = V_t.row(8).reshape(0, 3);
    cv::SVDecomp(F_p, D, U, V_t, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    //Enforce rank constraint
    D.at<float>(2,2) = 0;

    return U * cv::Mat::diag(D) * V_t;
}

float fundamentalError(const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::KeyPoint>& keypoints2, const cv::Mat& F, std::vector<bool>& inliers, int& nInliers, float thresh) {
    const int N = keypoints1.size();

    cv::Mat kp1_h = cv::Mat::ones(3, N, CV_32FC1);
    cv::Mat kp2_h = cv::Mat::ones(3, N, CV_32FC1);
    inliers.resize(N);
    nInliers = 0;

    for (int i = 0; i < N; i++) {
        kp1_h.at<float>(0, i) = keypoints1[i].pt.x;
        kp1_h.at<float>(1, i) = keypoints1[i].pt.y;
        kp2_h.at<float>(0, i) = keypoints2[i].pt.x;
        kp2_h.at<float>(1, i) = keypoints2[i].pt.y;
    }

    cv::Mat F_1 = F * kp1_h;
    cv::Mat F_t2 = F.t() * kp2_h;

    cv::Mat r;
    cv::reduce(kp2_h.mul(F_1), r, 0, cv::REDUCE_SUM);

    cv::Mat F_sq = F_1.row(0).mul(F_1.row(0)) + F_1.row(1).mul(F_1.row(1)) + \
            F_t2.row(0).mul(F_t2.row(0)) + F_t2.row(1).mul(F_t2.row(1));
    cv::sqrt(F_sq, F_sq);
    cv::Mat t = cv::abs(r) / F_sq;

    for (int i = 0; i < N; i++) {
        if (t.at<float>(i) < thresh) {
            inliers[i] = true;
            nInliers++;
        } else {
            inliers[i] = false;
        }
    }

    return cv::sum(t)[0];








    /* const int N = keypoints1.size(); */

    /* const float f00 = F.at<float>(0,0); */
    /* const float f01 = F.at<float>(0,1); */
    /* const float f02 = F.at<float>(0,2); */
    /* const float f10 = F.at<float>(1,0); */
    /* const float f11 = F.at<float>(1,1); */
    /* const float f12 = F.at<float>(1,2); */
    /* const float f20 = F.at<float>(2,0); */
    /* const float f21 = F.at<float>(2,1); */
    /* const float f22 = F.at<float>(2,2); */

    /* inliers.resize(N); */

    /* float score = 0; */

    /* const float th = 3.841; */
    /* const float thScore = 5.991; */

    /* const float invSigmaSquare = 1.0/(sigma*sigma); */

    /* for(int i=0; i<N; i++) */
    /* { */
    /*     bool bIn = true; */

    /*     const cv::Point2f &p1 = keypoints1[i].pt; */
    /*     const cv::Point2f &p2 = keypoints1[i].pt; */

    /*     const float u1 = p1.x; */
    /*     const float v1 = p1.y; */
    /*     const float u2 = p2.x; */
    /*     const float v2 = p2.y; */

    /*     // Reprojection error in second image */
    /*     // l2=F21x1=(a2,b2,c2) */

    /*     const float a2 = f00*u1+f01*v1+f02; */
    /*     const float b2 = f10*u1+f11*v1+f12; */
    /*     const float c2 = f20*u1+f21*v1+f22; */

    /*     const float num2 = a2*u2+b2*v2+c2; */

    /*     const float squareDist1 = num2*num2/(a2*a2+b2*b2); */

    /*     const float chiSquare1 = squareDist1*invSigmaSquare; */

    /*     if(chiSquare1>th) */
    /*         bIn = false; */
    /*     else */
    /*         score += thScore - chiSquare1; */

    /*     // Reprojection error in second image */
    /*     // l1 =x2tF21=(a1,b1,c1) */

    /*     const float a1 = f00*u2+f01*v2+f02; */
    /*     const float b1 = f10*u2+f11*v2+f12; */
    /*     const float c1 = f20*u2+f21*v2+f22; */

    /*     const float num1 = a1*u1+b1*v1+c1; */

    /*     const float squareDist2 = num1*num1/(a1*a1+b1*b1); */

    /*     const float chiSquare2 = squareDist2*invSigmaSquare; */

    /*     if(chiSquare2>th) */
    /*         bIn = false; */
    /*     else */
    /*         score += thScore - chiSquare2; */

    /*     if(bIn) */
    /*         inliers[i]=true; */
    /*     else */
    /*         inliers[i]=false; */
    /* } */

    /* return score; */
}
