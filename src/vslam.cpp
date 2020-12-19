#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#include "vslam_internal.h"
#include "RansacFilter.h"
#include "Display.h"
#include "helpers.h"
#include "Frame.h"
#include "PointMap.h"

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Expected video file name as a command line argument\n");
        return -1;
    }

    //TODO: refactor
    RansacFilter rf(8, 100, 10);

    /* cv::FileStorage fs("reprojected_points.json", cv::FileStorage::APPEND); */

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


    const u32 DISTANCE_THRESHOLD = 64;

    std::mutex mtx;

    PointMap pm;
    Display display("Map", W, H, &mtx);
    display.initialize();

    cv::Mat image;

    size_t frame_cnt = 0;
    const float thresholdSq = 4.0;
    std::vector<cv::Point3f> points;

    while (cap.isOpened()) {
        cap >> image;

        pm.frames.emplace_back();
        Frame &frame = pm.frames.back();
        initialize_frame(frame, image, frame_cnt);

        // NOTE(rahul): orb vs "good feature extraction (Shi-Tomasi Corner Detection)"
        /* extract_features(f, 5, 5); */
        extract_features(frame);

        if (frame_cnt == 0) {
            frame.pose = cv::Mat::eye(4, 4, CV_32F);
            frame.R_t = cv::Mat::eye(4, 4, CV_32F);
        } else {

            const Frame &last_frame = pm.frames[pm.frames.size() - 2];
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

            // NOTE(rahul): Try storing frame.pose as 16 floats and casting whenever we need to multiply (only here for now)
            frame.pose = last_frame.pose * frame.R_t;

            cv::Mat annotated;
            draw(frame, annotated);

            /* cv::Mat initial_points1(2, matches.size(), CV_32FC1); */
            /* cv::Mat initial_points2(2, matches.size(), CV_32FC1); */
            // TODO(rahul): make these interleaved
            cv::Mat initial_points1(matches.size(), 2, CV_32FC1);
            cv::Mat initial_points2(matches.size(), 2, CV_32FC1);
            f32 *initial_points_data1 = initial_points1.ptr<float>();
            f32 *initial_points_data2 = initial_points2.ptr<float>();
            for (memory_index i = 0, j = 0; i < matches.size(); i++, j += 2) {
                const std::pair<int, int>& m = matches[i];
                initial_points_data1[j]     = last_frame.points[m.first].x;
                initial_points_data1[j + 1] = last_frame.points[m.first].y;
                initial_points_data2[j]     = frame.points[m.second].x;
                initial_points_data2[j + 1] = frame.points[m.second].y;
                /* initial_points1.at<float>(0, i) = last_frame.points[m.first].x; */
                /* initial_points1.at<float>(1, i) = last_frame.points[m.first].y; */
                /* initial_points2.at<float>(0, i) = frame.points[m.second].x; */
                /* initial_points2.at<float>(1, i) = frame.points[m.second].y; */

                cv::line(annotated, last_frame.points[m.first], frame.points[m.second], cv::Scalar(0,0, 255));
            }

            cv::Mat c1 = cv::Mat::zeros(3, 4, CV_32FC1);
            K.copyTo(c1.colRange(0, 3));
            cv::Mat c2 = K * frame.R_t.rowRange(0, 3);

            /* //TODO: search by projection (assuming pm.points = Nx4) */
            /* cv::Mat projected_map_points = c2 * pm.points.rowRange(0, pm.size).t(); */
            if (pm.size > 0) {
                cv::Mat projected_map_points = pm.points.rowRange(0, pm.size) * c2.t();

                std::vector<bool> current_frame_inliers(pm.size, false);
                // make non homogeneous
                float *projected_map_points_data = projected_map_points.ptr<f32>();
                for (memory_index i = 0, j = 0; i < projected_map_points.rows; i++, j += 3) {
                    const f32 h = projected_map_points_data[j + 2];
                    f32 &x = projected_map_points_data[j + 0];
                    f32 &y = projected_map_points_data[j + 1];
                    x /= h;
                    y /= h;
                    if (x >= 0 && x < W && y >= 0 && y < H) {
                        current_frame_inliers[i] = true;
                    }
                }

                for (memory_index i = 0, j = 0; i < pm.size; i++, j += 3) {
                    if (!current_frame_inliers[i]) continue;
                    const cv::Point2f &query_pt = {projected_map_points_data[j], projected_map_points_data[j + 1]};
                    std::vector<memory_index> indices = radius_search(frame.kdtree, frame.points, query_pt, 2);
                    for (memory_index idx : indices) {
                        if (frame.map_point_ids[idx] >= 0) continue;
                        u32 dist = orb_distance(pm, i, frame, idx);
                        if (dist < DISTANCE_THRESHOLD) {
                            frame.map_point_ids[idx] = i;
                            pm.frame_ids[i].push_back(frame.id);
                            pm.frame_point_ids[i].push_back(idx);
                            break;
                        }
                    }
                }
            }
            /* for (int i = 0; i < projected_map_points.cols; i++) { */
            /*     const float h = projected_map_points.at<float>(2, i); */
            /*     const float x = (projected_map_points.at<float>(0, i) /= h); */
            /*     const float y = (projected_map_points.at<float>(1, i) /= h); */
            /*     if (x >= 0 && x < W && y >= 0 && y < H) { */
            /*         current_frame_inliers[i] = true; */
            /*     } */
            /* } */

            /* for (memory_index i = 0; i < pm.size; i++) { */
            /*     if (current_frame_inliers[i]) { */
            /*         const cv::Point2f query_pt = {projected_map_points.at<float>(0, i), (projected_map_points.at<float>(1, i))}; */
            /*         std::vector<cv::Point2f> pts = radius_search(frame.kdtree, query_pt, 2); */
            /*         for (auto &pt : pts) { */

            /*         } */
            /*     } */
            /* } */
            /* for (size_t i = 0; i < pm.frames.size(); i++) { */

            /* } */


            cv::Mat points_4d;
            triangulate(initial_points1, initial_points2, c1, c2, points_4d);

            // Reprojective error
            /* cv::Mat reproj_points1 = c1 * points_4d.t(); */
            /* cv::Mat reproj_points2 = c2 * points_4d.t(); */
            // NOTE(rahul): we do this as B^t * A^t to get (AB)^T (better for memory coherence)
            cv::Mat reproj_points1 = points_4d * c1.t();
            cv::Mat reproj_points2 = points_4d * c2.t();

            if (!reproj_points1.isContinuous() || !reproj_points2.isContinuous()) {
                fprintf(stderr, "Error: not continuous in %s, line %d\n", __FILE__, __LINE__);
            }

            f32 *reproj_points_data1 = reproj_points1.ptr<f32>();
            f32 *reproj_points_data2 = reproj_points2.ptr<f32>();
            for (memory_index i = 0; i < reproj_points1.rows; i += 3) {
                f32 &h1 = reproj_points_data1[i + 2];
                reproj_points_data1[i]     /= h1;
                reproj_points_data1[i + 1] /= h1;
                h1 = 1;

                f32 &h2 = reproj_points_data2[i + 2];
                reproj_points_data2[i]     /= h2;
                reproj_points_data2[i + 1] /= h2;
                h2 = 1;
            }
            /* char str[50]; */
            /* sprintf(str, "repro2%zu", frame_cnt); */
            /* write_matrix(reproj_points2, str, fs); */

            /* for (int i = 0; i < reproj_points2.cols; i++) { */
            /*     cv::Point2f pt(reproj_points2.at<float>(0, i), reproj_points2.at<float>(1, i)); */
            /*     cv::circle(annotated, pt, 2, cv::Scalar(255, 0, 0)); */
            /* } */
            for (memory_index i = 0; i < reproj_points2.rows; i += 3) {
                const cv::Point2f pt(reproj_points_data2[i], reproj_points_data2[i + 1]);
                /* const cv::Point2f pt(reproj_points2.at<float>(0, i), reproj_points2.at<float>(1, i)); */
                cv::circle(annotated, pt, 2, cv::Scalar(255, 0, 0));
            }

            /* reproj_points1 = reproj_points1.rowRange(0, 2); */
            /* reproj_points2 = reproj_points2.rowRange(0, 2); */
            reproj_points1 = reproj_points1.colRange(0, 2);
            reproj_points2 = reproj_points2.colRange(0, 2);

            cv::Mat d1 = reproj_points1 - initial_points1;
            cv::Mat d2 = reproj_points2 - initial_points2;

            f64 reproj_error = 0;
            std::vector<memory_index> reprojection_inliers;
            for (memory_index i = 0; i < d1.rows; i++) {
                f32 re1 = d1.row(i).dot(d1.row(i));
                if (re1 > thresholdSq) continue;
                f32 re2 = d2.row(i).dot(d2.row(i));
                if (re2 > thresholdSq) continue;
                else {
                    reprojection_inliers.push_back(i);
                    reproj_error += re1 + re2;
                }
            }

            /* for (int i = 0; i < d1.cols; i++) { */
            /*     float re1 = d1.col(i).dot(d1.col(i)); */
            /*     if (re1 > thresholdSq) continue; */
            /*     float re2 = d2.col(i).dot(d2.col(i)); */
            /*     if (re2 > thresholdSq) continue; */
            /*     else { */
            /*         reprojection_inliers.push_back(i); */
            /*         reproj_error += re1 + re2; */
            /*     } */
            /* } */

            mtx.lock();

            add_reprojection_inliers(pm, points_4d, reprojection_inliers);

            /* for (int i : reprojection_inliers) { */
            /*     points.emplace_back(points_4d.at<float>(0, i), points_4d.at<float>(1, i), points_4d.at<float>(2, i)); */
            /* } */

            display.ds.points = &pm.points;
            display.ds.size = pm.size;
            display.ds.frames = &pm.frames;
            mtx.unlock();

            printf("Reprojection inliers: %zu\n", reprojection_inliers.size());
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
    for (auto &f : pm.frames) {
        free(f.kdtree.root);
    }
    display.join();
    return 0;
}
