#include "Frame.h"
#include "MapPoint.h"

Frame::Frame(const cv::Mat& image, const cv::Mat& K, const cv::Mat pose) :
K(K), pose(pose) {
    cv::invert(K, K_inv);
    if (!image.empty()) {
        extract_key_points(image, keypoints, descriptors);
        //TODO: NORMALIZE KEY POINTS

        mapPoints.resize(keypoints.size(), NULL);
    }
}

void Frame::draw(const cv::Mat& image, cv::Mat& drawn) {
    image.copyTo(drawn);
    for (size_t i = 0; i < keypoints.size(); i++) {
        cv::Point2f pt = keypoints[i].pt;
        int u = (int) round(pt.x), v = (int) round(pt.y);
        if (mapPoints[i] != NULL) {
            if (mapPoints[i]->frames.size() >= 5) {
                cv::circle(drawn, cv::Point(u, v), 3, cv::Scalar(0, 255, 0));
            } else {
                cv::circle(drawn, cv::Point(u, v), 3, cv::Scalar(0, 128, 0));
            }
            /* std::vector<cv::Point> */
        }
    }
}

void Frame::undistort_points() {
    /* cv::Mat pts_h(points); */
    cv::Mat pts_h = cv::Mat::ones(3, keypoints.size(), CV_32FC1);
    for (size_t i = 0; i < keypoints.size(); i++) {
        pts_h.at<float>(0, i) = keypoints[i].pt.x;
        pts_h.at<float>(1, i) = keypoints[i].pt.y;
    }
    cv::Mat u = K_inv * pts_h;
    undistorted_points.resize(keypoints.size());
    for (size_t i = 0; i < undistorted_points.size(); i++) {
        float h = pts_h.at<float>(2, i);
        undistorted_points[i] = cv::Point2f(pts_h.at<float>(0, i)/h, pts_h.at<float>(1, i)/h);
    }

}

void Frame::generate_kdtree() {
    cv::Mat_<float> features(0, 2);
    for (auto& point : keypoints) {
        cv::Mat row = (cv::Mat_<float>(1, 2) << point.pt.x, point.pt.y);
        features.push_back(row);
    }
    kdtree = cv::flann::Index(features, cv::flann::KDTreeIndexParams(1));
}
