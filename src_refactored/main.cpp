#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include "RansacFilter.h"

//TODO: refactor
RansacFilter rf(8, 100, 10);
//TODO: calibrate camera
cv::Mat K(3, 3, CV_32FC1, (float[9]) {200, 0, 960, 0, 200, 540, 0, 0, 1});
cv::Mat K_inv;


void extract_pose(cv::Mat *F, cv::Mat *pose) {
    cv::Mat E = K.t() * (*F) * K;

    cv::Mat U, D, V_t;
    cv::SVD::compute(E, D, U, V_t);

    cv::Mat t;
    U.col(2).copyTo(t);
    //TODO: investigate if norm is good or bad , t might be negative as well
    t /= cv::norm(t);

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
    cv::Mat R = (R_1.at<float>(0, 0) + R_1.at<float>(1, 1) + R_1.at<float>(2, 2) < 0) ? R_2 : R_1;
    //TODO: what?
    if (t.at<float>(2) < 0) {
        t *= -1;
    }

    //for now
    (*pose) = cv::Mat::eye(4, 4, CV_32FC1);
    R.copyTo((*pose).rowRange(0, 3).colRange(0, 3));
    t.copyTo((*pose).rowRange(0, 3).col(3));



}

//TODO: why is orb extraction so much worse?
void extract_key_points(cv::Mat& image, std::vector<cv::KeyPoint> *keypoints, cv::Mat *descriptors, const int ncols, const int nrows) {
    //TODO: Try Different KP extractors

    const int nfeatures = 500;
    /* const int cells = ncols * nrows; */
    const int cw = image.cols / ncols, ch = image.rows / nrows;
    /* cv::Ptr<cv::ORB> orb_feature_detector = cv::ORB::create(nfeatures); */
    cv::Ptr<cv::ORB> orb_feature_detector = cv::ORB::create(nfeatures, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
    cv::Ptr<cv::ORB> fallback_feature_detector = cv::ORB::create(nfeatures, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 5);

    for (int i = 0; i < ncols; i++) {
        for (int j = 0; j < nrows; j++) {
            std::vector<cv::KeyPoint> temp;
            int sx = i * cw, sy = j * ch;
            cv::Rect rect(sx, sy, cw, ch);
            cv::rectangle(image, rect, cv::Scalar(0, 0, 0));
            orb_feature_detector->detect(image(rect), temp);
            if (temp.size() < nfeatures) {
                fallback_feature_detector->detect(image(rect), temp);
            }
            for (auto &p : temp) {
                keypoints->emplace_back(sx + p.pt.x, sy + p.pt.y, p.size, p.angle, p.response, p.octave, p.class_id);
            }
        }
    }
    orb_feature_detector->compute(image, *keypoints, *descriptors);
    /* printf("# Keypoints found in first image: %zu\n", keypoints->size()); */
}

void extract_key_points(cv::Mat& image, std::vector<cv::KeyPoint> *keypoints, cv::Mat *descriptors) {

    cv::Ptr<cv::ORB> orb_feature_detector = cv::ORB::create();
    std::vector<cv::Point2f> points;
    cv::goodFeaturesToTrack(image, points, 3000, 0.01, 3);
    keypoints->reserve(points.size());
    for (cv::Point2f p : points) {
        keypoints->emplace_back(p, 20);
    }
    orb_feature_detector->compute(image, *keypoints, *descriptors);
    /* printf("# Keypoints found in first image: %zu\n", keypoints->size()); */
}

void match_features(std::vector<cv::KeyPoint> *keypoints1, std::vector<cv::KeyPoint> *keypoints2, cv::Mat *descriptors1, cv::Mat *descriptors2, std::vector<std::pair<int, int>> *matches, cv::Mat *F) {
    cv::Ptr<cv::BFMatcher> bfmatcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> initialMatches;
    bfmatcher->knnMatch(*descriptors1, *descriptors2, initialMatches, 2);

    std::vector<std::pair<int, int>> i_matches;

    for (std::vector<cv::DMatch> m : initialMatches) {
        //lowes
        if (m[0].distance < m[1].distance * 0.7) {
            i_matches.push_back(std::make_pair(m[0].queryIdx, m[0].trainIdx));
        }
    }


    rf.initialize(keypoints1, keypoints2, &i_matches);
    std::vector<bool> inliers;
    rf.find_fundamental(&inliers, F);
    for (size_t i = 0; i < inliers.size(); i++) {
        if (inliers[i]) {
            matches->emplace_back(i_matches[i]);
        }
    }
    /* printf("%zu keypoints -> %zu preliminary matches -> %zu final matches\n", keypoints1->size(), i_matches.size(), matches->size()); */


}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Expected video file name as a command line argument\n");
        return -1;
    }

    const char * vName = argv[1];
    cv::VideoCapture cap(vName);
    int W = (int) cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int H = (int) cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    cv::invert(K, K_inv);
    std::cout << "K_inv" << '\n' << K_inv << '\n' << '\n';

    cv::Mat frame;
    std::vector<cv::KeyPoint> lastKeypoints;
    cv::Mat lastDescriptors;

    cv::Mat Rt = cv::Mat::eye(4, 4, CV_32FC1);
    size_t frame_cnt = 0;

    std::vector<cv::Mat *> Rts;
    while (cap.isOpened()) {
        cap >> frame;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        extract_key_points(gray, &keypoints, &descriptors);
        /* extract_key_points(frame, &keypoints, &descriptors, 5, 5); */

        if (frame_cnt > 0) {
            std::vector<std::pair<int, int>> matches;
            //match
            cv::Mat fundamental;
            match_features(&lastKeypoints, &keypoints, &lastDescriptors, &descriptors, &matches, &fundamental); // we match from last frame to next frame (this = x2)
            cv::Mat pose;
            extract_pose(&fundamental, &pose);
            Rt = pose * Rt;
            Rts.emplace_back(&Rt);

            std::cout << "Rt" << '\n' << *Rts.back() << '\n' << '\n';

            cv::Mat annotated = frame.clone();
            for (auto p : keypoints) {
                cv::circle(annotated, p.pt, 2, cv::Scalar(0, 255, 0));
            }
            for (auto m : matches) {
                cv::line(annotated, lastKeypoints[m.first].pt, keypoints[m.second].pt, cv::Scalar(0,0, 255));
                /* cv::circle(annotated, p.pt, 2, cv::Scalar(0, 255, 0)); */
            }

            cv::imshow("points", annotated);
            if (cv::waitKey(25) == 113) {
                break;
            }
        }

        lastKeypoints.swap(keypoints);
        lastDescriptors = descriptors;
        frame_cnt++;
    }
}
