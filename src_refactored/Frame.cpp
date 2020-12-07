#include "Frame.h"

void initialize_frame(Frame &frame, const cv::Mat &image) {
    frame.image = image;
}

void draw(const Frame &frame, cv::Mat &annotated) {
    frame.image.copyTo(annotated);
    for (auto &kp : frame.keypoints) {
        cv::circle(annotated, kp.pt, 2, cv::Scalar(0, 255, 0));
    }
}

//TODO: why is orb extraction so much slower/worse?
void extract_features(Frame &frame, int nrows, int ncols) {
    //TODO: Try Different KP extractors

    const int nfeatures = 500;
    const int cw = frame.image.cols / ncols, ch = frame.image.rows / nrows;
    //TODO; decrease wasted areas inside grid
    cv::Ptr<cv::ORB> orb_feature_detector = cv::ORB::create(nfeatures, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
    cv::Ptr<cv::ORB> fallback_feature_detector = cv::ORB::create(nfeatures, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 5);

    for (int i = 0; i < ncols; i++) {
        for (int j = 0; j < nrows; j++) {
            std::vector<cv::KeyPoint> temp;
            int sx = i * cw, sy = j * ch;
            cv::Rect rect(sx, sy, cw, ch);
            cv::rectangle(frame.image, rect, cv::Scalar(0, 0, 0));
            orb_feature_detector->detect(frame.image(rect), temp);
            if (temp.size() < nfeatures) {
                fallback_feature_detector->detect(frame.image(rect), temp);
            }
            for (auto &p : temp) {
                frame.keypoints.emplace_back(sx + p.pt.x, sy + p.pt.y, p.size, p.angle, p.response, p.octave, p.class_id);
            }
        }
    }
    orb_feature_detector->compute(frame.image, frame.keypoints, frame.descriptors);
    /* printf("# Keypoints found in first image: %zu\n", keypoints->size()); */
}

void extract_features(Frame &frame) {

    cv::Mat gray;
    cv::cvtColor(frame.image, gray, cv::COLOR_BGR2GRAY);
    cv::Ptr<cv::ORB> orb_feature_detector = cv::ORB::create();
    std::vector<cv::Point2f> points;
    cv::goodFeaturesToTrack(gray, points, 3000, 0.01, 3);
    frame.keypoints.reserve(points.size());
    for (cv::Point2f p : points) {
        frame.keypoints.emplace_back(p, 20);
    }
    orb_feature_detector->compute(gray, frame.keypoints, frame.descriptors);
    /* printf("# Keypoints found in first image: %zu\n", keypoints->size()); */
}

void match_features(const Frame &frame1, const Frame &frame2, RansacFilter &rf, std::vector<std::pair<int, int>> &matches, cv::Mat &F) {
    cv::Ptr<cv::BFMatcher> bfmatcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> initialMatches;
    bfmatcher->knnMatch(frame1.descriptors, frame2.descriptors, initialMatches, 2);

    std::vector<std::pair<int, int>> i_matches;

    for (std::vector<cv::DMatch> m : initialMatches) {
        //lowes
        if (m[0].distance < m[1].distance * 0.7) {
            i_matches.push_back(std::make_pair(m[0].queryIdx, m[0].trainIdx));
        }
    }

    std::vector<bool> inliers;
    rf.find_fundamental(frame1.keypoints, frame2.keypoints, i_matches, inliers, F);
    for (size_t i = 0; i < inliers.size(); i++) {
        if (inliers[i]) {
            matches.emplace_back(i_matches[i]);
        }
    }
    //TODO: cross checking/kdtree optimization
    /* printf("%zu keypoints -> %zu preliminary matches -> %zu final matches\n", frame1.keypoints.size(), i_matches.size(), matches.size()); */
}
