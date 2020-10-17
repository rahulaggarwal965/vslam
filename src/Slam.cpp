#include "Slam.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/matx.hpp"
#include "opencv2/core/types.hpp"
#include <exception>

Slam::Slam(int W, int H, const cv::Mat& K) : point_map(), K(K), W(W), H(H) {}

void Slam::process_frame(const cv::Mat &image) {
    Frame frame(image, K);
    frame.id = point_map->add_frame(frame);

    if (frame.id == 0) return;

    size_t num_frames = point_map->frames.size();
    Frame *frame1 = point_map->frames[num_frames - 1];
    Frame *frame2 = point_map->frames[num_frames - 2];

    std::vector<int> idx1, idx2;
    cv::Mat pose, Rt_i;

    match_frames(*frame1, *frame2, K, idx1, idx2, pose);

    for (size_t i = 0; i < idx2.size(); i++) {
        if (frame2->mapPoints[idx2[i]] != NULL && frame1->mapPoints[idx1[i]] == NULL) {
            frame2->mapPoints[idx2[i]]->add_observation(*frame1, idx1[i]);
        }
    }


    //TODO: switch so no inversion?
    cv::invert(pose, Rt_i);
    /* if (frame.id < 5) { */
    frame1->pose = Rt_i * frame2->pose;
    /* } else { */
        //TODO: Kinematic Model, constant velocity
    /* } */

    int projection_point_count = 0;

    if (size_t s = point_map->mapPoints.size() > 0) {
        //TODO: refactor to matrix operations (don't use homogeneous)
        cv::Mat mapPoints(3, s, CV_32FC1);
        for (size_t i = 0; i < s; i++) {
            mapPoints.col(i) = point_map->mapPoints[i]->homogeneous();
        }
        cv::Mat projections = (K * frame1->pose(cv::Range(0, 3), cv::Range(0, 3))) * mapPoints;
        std::vector<cv::Vec2f> projectionPoints;
        std::vector<bool> goodPoints;

        projectionPoints.reserve(s);
        goodPoints.reserve(s);
        for (size_t i = 0; i < s; i++) {
            cv::Vec3f h = mapPoints.col(i);
            h /= h[2];
            projectionPoints.emplace_back(h[0], h[1]);
            if (h[0] > 0 && h[0] < W && h[1] > 0 && h[1] < H) {
                goodPoints.push_back(true);
            } else {
                goodPoints.push_back(false);
            }
        }
        frame1->generate_kdtree();
        for (size_t i = 0; i < s; i++) {
            if (!goodPoints[i]) continue;
            if (std::find(point_map->mapPoints[i]->frames.begin(), point_map->mapPoints[i]->frames.end(), frame1) != point_map->mapPoints[i]->frames.end()) {
                continue;
            }
            std::vector<int> indices;
            frame1->kdtree.radiusSearch(projectionPoints[i], indices, cv::Mat(), 2, 500);
            for (size_t j = 0; j < indices.size(); j++) {
                if (frame1->mapPoints[indices[j]] == NULL) {
                    double dist = point_map->mapPoints[i]->orb_distance(frame1->descriptors.row(indices[j]));
                    if (dist < 64.0) {
                        point_map->mapPoints[i]->add_observation(*frame1, indices[j]);
                        projection_point_count++;
                        break;
                    }
                }
            }
        }

        std::vector<bool> others;
        others.reserve(idx1.size());
        for (int i : idx1) {
            if (frame1->mapPoints[i] == NULL) {
                others.push_back(true);
            } else {
                others.push_back(false);
            }
        }

        cv::Mat tPoints1(2, idx1.size(), CV_32FC1), tPoints2(2, idx2.size(), CV_32FC1);
        //TODO: use normalized version
        for (size_t i = 0; i < idx1.size(); i++) {
            auto& point = frame1->keypoints[idx1[i]];
            tPoints1.col(i) = cv::Vec2f(point.pt.x, point.pt.y);
        }
        for (size_t i = 0; i < idx2.size(); i++) {
            auto& point = frame2->keypoints[idx2[i]];
            tPoints2.col(i) = cv::Vec2f(point.pt.x, point.pt.y);
        }

        cv::Mat points_4d;
        cv::triangulatePoints(frame1->pose.rowRange(0, 3), frame2->pose.rowRange(0, 3), tPoints1, tPoints2, points_4d);

        for (size_t i = 0; i < others.size(); i++) {
            float h = points_4d.at<float>(3, i);
            others[i] = others[i] && (abs(h) != 0);
            points_4d.col(i).rowRange(0, 3) /= h;
        }

        int np_count = 0;
        for (int i = 0; i < points_4d.cols; i++) {
            if (!others[i]) continue;
            cv::Mat p11 = frame1->pose * points_4d.col(i);
            cv::Mat p12 = frame2->pose * points_4d.col(i);
            if (p11.at<float>(2) < 0 || p12.at<float>(2) < 0) continue;

            //Reprojecting using intrinsic
            cv::Mat projected_p1 = K * p11.rowRange(0, 3);
            cv::Mat projected_p2 = K * p12.rowRange(0, 3);


            //Checking the reprojective error
            cv::Point pt = frame1->keypoints[idx1[i]].pt;
            cv::Mat temp = (cv::Mat_<float>(2, 1) << pt.x, pt.y);
            projected_p1 = (projected_p1.rowRange(0, 2) / projected_p1.row(2)) - temp;
            pt = frame2->keypoints[idx2[i]].pt;
            temp = (cv::Mat_<float>(2, 1) << pt.x, pt.y);
            projected_p2 = (projected_p2.rowRange(0, 2) / projected_p2.row(2)) - temp;
            float p1_sum = projected_p1.dot(projected_p1);
            float p2_sum = projected_p2.dot(projected_p2);
            if (p1_sum > 2 || p2_sum > 2) continue;

            int x = (int) round(pt.x);
            int y = (int) round(pt.y);
            cv::Scalar color(0, 0, 255);
            if (x > 0 && x < image.cols && y > 0 && y < image.rows) {
                color = image.at<cv::Scalar>(y, x);
            }
            MapPoint p(point_map, points_4d.col(i).rowRange(0, 3), color);
            p.add_observation(*frame2, idx2[i]);
            p.add_observation(*frame1, idx1[i]);
            np_count++;

            //TODO: Optimize
        }

    }

}
