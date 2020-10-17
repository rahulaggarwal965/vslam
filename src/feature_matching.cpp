#include "feature_matching.h"
#include "opencv2/calib3d.hpp"
#include "transformation_tools.h"

#define M_DISTANCE_RATIO 0.7

void extract_key_points(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
    //TODO: Try Different KP extractors
    //TODO: Try different orb parameters
    cv::Ptr<cv::ORB> orb_feature_detector = cv::ORB::create(3000);

    orb_feature_detector->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
    /* printf("# Keypoints found in first image: %zu", keypoints.size()); */
}

//TODO: refactor, very inefficient
void match_frames(const Frame& frame1, const Frame& frame2, const cv::Mat& K, std::vector<int>& idx1, std::vector<int>& idx2, cv::Mat& pose) {
    cv::Ptr<cv::BFMatcher> bfmatcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> initialMatches;
    bfmatcher->knnMatch(frame1.descriptors, frame2.descriptors, initialMatches, 2);

    std::set<int> sIndexes1, sIndexes2;
    std::vector<int> indexes1, indexes2;
    std::vector<cv::Point2f> matched_kp1, matched_kp2;
    /* std::vector<cv::Vec<cv::KeyPoint, 2>> r; */

    for (auto m : initialMatches) {
        if (m[0].distance < m[1].distance * M_DISTANCE_RATIO) {
            cv::KeyPoint kp1 = frame1.keypoints[m[0].queryIdx];
            cv::KeyPoint kp2 = frame2.keypoints[m[0].trainIdx];

            //TODO: Tune distance threshold
            if (m[0].distance < 32) {
                //Refactor
                if (sIndexes1.find(m[0].queryIdx) == sIndexes1.end() && sIndexes2.find(m[0].trainIdx) == sIndexes2.end()) {
                    indexes1.push_back(m[0].queryIdx);
                    indexes2.push_back(m[0].trainIdx);
                    sIndexes1.insert(m[0].queryIdx);
                    sIndexes2.insert(m[0].trainIdx);
                    matched_kp1.push_back(kp1.pt);
                    matched_kp2.push_back(kp2.pt);
                }
            }
        }
    }
    // We don't want duplicates
    assert(sIndexes1.size() == indexes1.size());
    assert(sIndexes2.size() == indexes2.size());

    cv::Mat mask;
    cv::Mat fundamental_matrix;
    //TODO: tune parameters
    //Must be 8 or more matches
    //Fundamental
    fundamental_matrix = cv::findFundamentalMat(matched_kp1, matched_kp2, cv::FM_RANSAC, 3, 0.99, mask);
    fundamentalMatrixToPose(fundamental_matrix, K, pose);

    idx1.reserve(idx1.size());
    idx2.reserve(idx2.size());
    for (int i = 0; i < mask.rows; i++) {
        if ((unsigned int)mask.at<uchar>(i)) {
            idx1.push_back(indexes1[i]);
            idx2.push_back(indexes2[i]);
        }
    }
    printf("Matches: %d -> %zu -> %zu\n", frame1.descriptors.rows, initialMatches.size(), idx1.size());
}
