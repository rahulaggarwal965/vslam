#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "RansacFilter.h"
#include "Display.h"
#include "helpers.h"
#include "Frame.h"

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Expected video file name as a command line argument\n");
        return -1;
    }

    //TODO: refactor
    RansacFilter rf(8, 100, 10);

    cv::FileStorage fs("reprojected_points.json", cv::FileStorage::APPEND);

    const char * vName = argv[1];
    cv::VideoCapture cap(vName);
    const int W = (int) cap.get(cv::CAP_PROP_FRAME_WIDTH);
    const int H = (int) cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    //TODO: calibrate camera
    const char *f_str = std::getenv("F");
    const float f = (f_str == NULL) ? 525 : atof(f_str);
    printf("f: %f\n", f);
    float k_data[9] = {f, 0, static_cast<float>(W / 2), 0, f, static_cast<float>(H / 2), 0, 0, 1};
    cv::Mat K(3, 3, CV_32FC1, k_data);
    cv::Mat K_inv;
    cv::invert(K, K_inv);
    print_matrix(K_inv, "K_inv");

    std::mutex mtx;

    std::vector<Frame> frames;
    Display display("Map", W, H, &mtx);
    display.initialize();

    cv::Mat image;

    size_t frame_cnt = 0;
    const float thresholdSq = 4.0;
    std::vector<cv::Point3f> points;

    while (cap.isOpened()) {
        cap >> image;

        frames.emplace_back();
        Frame &frame = frames.back();
        initialize_frame(frame, image);
        /* extract_features(f, 5, 5); */
        extract_features(frame);

        if (frame_cnt == 0) {
            frame.pose = cv::Mat::eye(4, 4, CV_32F);
            frame.R_t = cv::Mat::eye(4, 4, CV_32F);
        } else {

            const Frame &last_frame = frames[frames.size() - 2];
            std::vector<std::pair<int, int>> matches;
            //match
            cv::Mat fundamental;
            // we match from last frame to next frame (this = x2)
            match_features(last_frame, frame, rf, matches, fundamental);
            // match_features(last_keypoints, keypoints, lastDescriptors, descriptors, rf, matches, fundamental);


            // Relative rotation (from last frame to this)
            cv::Mat rotation, translation;
            extract_Rt(fundamental, K, rotation, translation);
            frame.R_t = cv::Mat::eye(4, 4, CV_32F);
            rotation.copyTo(frame.R_t.rowRange(0, 3).colRange(0, 3));
            translation.copyTo(frame.R_t.rowRange(0, 3).col(3));
            frame.pose = last_frame.pose * frame.R_t;

            cv::Mat annotated;
            draw(frame, annotated);

            cv::Mat initial_points1(2, matches.size(), CV_32FC1);
            cv::Mat initial_points2(2, matches.size(), CV_32FC1);
            for (size_t i = 0; i < matches.size(); i++) {
                const std::pair<int, int>& m = matches[i];
                initial_points1.at<float>(0, i) = last_frame.keypoints[m.first].pt.x;
                initial_points1.at<float>(1, i) = last_frame.keypoints[m.first].pt.y;
                initial_points2.at<float>(0, i) = frame.keypoints[m.second].pt.x;
                initial_points2.at<float>(1, i) = frame.keypoints[m.second].pt.y;

                cv::line(annotated, last_frame.keypoints[m.first].pt, frame.keypoints[m.second].pt, cv::Scalar(0,0, 255));
            }

            //TODO: triangulation

            cv::Mat c1 = cv::Mat::zeros(3, 4, CV_32FC1);
            K.copyTo(c1.colRange(0, 3));
            cv::Mat c2 = K * frame.R_t.rowRange(0, 3);

            cv::Mat points_4d;
            triangulate(initial_points1, initial_points2, c1, c2, points_4d);

            // Reprojective error
            cv::Mat reproj_points1 = c1 * points_4d;
            cv::Mat reproj_points2 = c2 * points_4d;

            for (int i = 0; i < reproj_points1.cols; i++) {
                reproj_points1.col(i) /= reproj_points1.at<float>(2, i);
                reproj_points2.col(i) /= reproj_points2.at<float>(2, i);
            }
            reproj_points1 = reproj_points1.rowRange(0, 2);
            reproj_points2 = reproj_points2.rowRange(0, 2);

            for (int i = 0; i < reproj_points2.cols; i++) {
                cv::Point2f pt(reproj_points2.at<float>(0, i), reproj_points2.at<float>(1, i));
                cv::circle(annotated, pt, 2, cv::Scalar(255, 0, 0));
            }

            cv::Mat d1 = reproj_points1 - initial_points1;
            cv::Mat d2 = reproj_points2 - initial_points2;


            float reproj_error = 0;
            std::vector<int> reprojection_inliers;
            for (int i = 0; i < d1.cols; i++) {
                float re1 = d1.col(i).dot(d1.col(i));
                if (re1 > thresholdSq) continue;
                float re2 = d2.col(i).dot(d2.col(i));
                if (re2 > thresholdSq) continue;
                else {
                    reprojection_inliers.push_back(i);
                    reproj_error += re1 + re2;
                }
            }

            mtx.lock();
            for (int i : reprojection_inliers) {
                points.emplace_back(points_4d.at<float>(0, i), points_4d.at<float>(1, i), points_4d.at<float>(2, i));
            }
            display.ds.points = &points;
            display.ds.frames = &frames;
            mtx.unlock();

            /* printf("Reprojection inliers: %zu\n", reprojection_inliers.size()); */
            /* printf("Reprojection Error: %f\n", reproj_error); */


            /* char str[50]; */
            /* sprintf(str, "diff%zu", frame_cnt); */
            /* write_matrix(d1, str, fs); */

            cv::imshow("points", annotated);
            if (cv::waitKey(1) == 113) {
                display.close();
                break;
            }
        }

        frame_cnt++;
    }
    display.join();
    return 0;
}
