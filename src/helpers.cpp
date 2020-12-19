#include "helpers.h"

void extract_Rt(const cv::Mat &fundamental, const cv::Mat &K, cv::Mat &rotation, cv::Mat &translation) {
    cv::Mat E = K.t() * fundamental * K;

    cv::Mat U, D, V_t;
    cv::SVD::compute(E, D, U, V_t);

    U.col(2).copyTo(translation);
    //TODO: investigate if norm is good or bad , t might be negative as well
    translation /= cv::norm(translation);

    cv::Mat W = cv::Mat::zeros(3, 3, CV_32FC1);
    W.at<float>(0, 1) = -1;
    W.at<float>(1, 0) = 1;
    W.at<float>(2, 2) = 1;

    cv::Mat R_1 = U * W * V_t;
    if (cv::determinant(R_1) < 0) {
        R_1 = -R_1;
    }

    cv::Mat R_2 = U * W.t() * V_t;
    if (cv::determinant(R_2) < 0) {
        R_2 = -R_2;
    }

    //TODO: better way of checking which rotation matrix is better
    rotation = (R_1.at<float>(0, 0) + R_1.at<float>(1, 1) + R_1.at<float>(2, 2) < 0) ? R_2 : R_1;
    //TODO: what?
    if (translation.at<float>(2) < 0) {
        translation *= -1;
    }

}

void triangulate(const cv::Mat &p1, const cv::Mat &p2, const cv::Mat &c1, const cv::Mat &c2, cv::Mat &points_4d) {
    cv::Mat A(4, 4, CV_32F);
    const memory_index N = p1.rows;
    /* points_4d = cv::Mat(4, N, CV_32F); */
    points_4d = cv::Mat(N, 4, CV_32F);
    f32 *p4d_data = points_4d.ptr<f32>();

    const f32 *p1_data = p1.ptr<f32>();
    const f32 *p2_data = p2.ptr<f32>();

    /* printf("Least Singular Values: "); */
    for (memory_index i = 0, j = 0, k = 0; i < N; i++, j += 2, k += 4) {
        A.row(0) = p1_data[j]     * c1.row(2) - c1.row(0);
        A.row(1) = p1_data[j + 1] * c1.row(2) - c1.row(1);
        A.row(2) = p2_data[j]     * c2.row(2) - c2.row(0);
        A.row(3) = p2_data[j + 1] * c2.row(2) - c2.row(1);
        /* A.row(0) = p1.at<float>(i, 0) * c1.row(2) - c1.row(0); */
        /* A.row(1) = p1.at<float>(i, 1) * c1.row(2) - c1.row(1); */
        /* A.row(2) = p2.at<float>(i, 0) * c2.row(2) - c2.row(0); */
        /* A.row(3) = p2.at<float>(i, 1) * c2.row(2) - c2.row(1); */

        cv::Mat U, D, V_t;
        cv::SVD::compute(A, D, U, V_t, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
        /* printf("%f ", D.at<float>(3)); */
        /* points_4d.col(i) = V_t.row(3).t(); */
        /* points_4d.col(i) /= points_4d.at<float>(3, i); */

        /* points_4d.row(i) = Va */

        if (!V_t.isContinuous()) {
            fprintf(stderr, "Error: not continuous in %s, line %d\n", __FILE__, __LINE__);
        }
        const f32 *V_t_data = V_t.ptr<f32>(3);

        /* f32 &w = p4d_data[k + 3]; */
        p4d_data[k]     = V_t_data[0] / V_t_data[3];
        p4d_data[k + 1] = V_t_data[1] / V_t_data[3];
        p4d_data[k + 2] = V_t_data[2] / V_t_data[3];
        p4d_data[k + 3] = 1;
    }
    /* printf("\n"); */
    /* printf("Continuous?: %d\n", points_4d.isContinuous()); */
    /* points_3d = points_3d.rowRange(0, 3); */
}

/* void make_homogeneous(const cv::Mat &m, cv::Mat &H) { */
/*     cv::Mat() */
/* } */
