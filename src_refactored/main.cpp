#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "RansacFilter.h"
/* #include "Display.h" */
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

    std::vector<Frame> frames;

    cv::Mat image;

    size_t frame_cnt = 0;

    while (cap.isOpened()) {
        cap >> image;

        frames.emplace_back();
        Frame &f = frames.back();
        initialize_frame(f, image);
        /* extract_features(f, 5, 5); */
        extract_features(f);

        if (frame_cnt == 0) {
            f.pose = cv::Mat::eye(4, 4, CV_32F);
            f.R_t = cv::Mat::eye(4, 4, CV_32F);
        } else {

            const Frame &last_frame = frames[frames.size() - 2];
            std::vector<std::pair<int, int>> matches;
            //match
            cv::Mat fundamental;
            // we match from last frame to next frame (this = x2)
            match_features(last_frame, f, rf, matches, fundamental);
            // match_features(last_keypoints, keypoints, lastDescriptors, descriptors, rf, matches, fundamental);


            // Relative rotation (from last frame to this)
            cv::Mat rotation, translation;
            extract_Rt(fundamental, K, rotation, translation);
            f.R_t = cv::Mat::eye(4, 4, CV_32F);
            rotation.copyTo(f.R_t.rowRange(0, 3).colRange(0, 3));
            translation.copyTo(f.R_t.rowRange(0, 3).col(3));
            f.pose = last_frame.pose * f.R_t;

            cv::Mat annotated;
            draw(f, annotated);

            cv::Mat initial_points1(2, matches.size(), CV_32FC1);
            cv::Mat initial_points2(2, matches.size(), CV_32FC1);
            for (size_t i = 0; i < matches.size(); i++) {
                const std::pair<int, int>& m = matches[i];
                initial_points1.at<float>(0, i) = last_frame.keypoints[m.first].pt.x;
                initial_points1.at<float>(1, i) = last_frame.keypoints[m.first].pt.y;
                initial_points2.at<float>(0, i) = f.keypoints[m.second].pt.x;
                initial_points2.at<float>(1, i) = f.keypoints[m.second].pt.y;
                /* temp_matches[0].push_back(last_keypoints[m.first].pt); */
                /* temp_matches[1].push_back(keypoints[m.second].pt); */
                cv::line(annotated, last_frame.keypoints[m.first].pt, f.keypoints[m.second].pt, cv::Scalar(0,0, 255));
                /* cv::circle(annotated, p.pt, 2, cv::Scalar(0, 255, 0)); */
            }

            //TODO: triangulation

            // Relative camera matrices
            /* cv::Mat c1 = K * Rts[Rts.size() - 2]->rowRange(0, 3); */
            /* cv::Mat c2 = K * Rts.back()->rowRange(0, 3); */

            cv::Mat c1 = cv::Mat::zeros(3, 4, CV_32FC1);
            K.copyTo(c1.colRange(0, 3));
            cv::Mat c2 = K * f.R_t.rowRange(0, 3);

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
            for (int i = 0; i < d1.cols; i++) {
               reproj_error += d1.col(i).dot(d1.col(i)) + d2.col(i).dot(d2.col(i));
            }

            printf("Reprojection Error: %f\n", reproj_error);


            /* char str[50]; */
            /* sprintf(str, "diff%zu", frame_cnt); */
            /* write_matrix(d1, str, fs); */

            cv::imshow("points", annotated);
            if (cv::waitKey(25) == 113) {
                /* display.close(); */
                break;
            }
        }

        frame_cnt++;
    }
    /* display.join(); */
    return 0;
}
