#include "opencv2/core/types.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

//TODO: why is orb extraction so much worse?
void extract_key_points(cv::Mat& image, std::vector<cv::KeyPoint> *keypoints, cv::Mat *descriptors, const int ncols, const int nrows) {
    //TODO: Try Different KP extractors

    const int nfeatures = 500;
    const int cells = ncols * nrows;
    const int cw = image.cols / ncols, ch = image.rows / nrows;
    cv::Ptr<cv::ORB> orb_feature_detector = cv::ORB::create(nfeatures);
    /* cv::Ptr<cv::ORB> orb_feature_detector = cv::ORB::create(nfeatures / cells, 1.2f, 8, 15, 0, 2, cv::ORB::HARRIS_SCORE, 15, 20); */
    /* cv::Ptr<cv::ORB> fallback_feature_detector = cv::ORB::create(nfeatures / cells, 1.2f, 8, 15, 0, 2, cv::ORB::HARRIS_SCORE, 15, 5); */

    for (int i = 0; i < ncols; i++) {
        for (int j = 0; j < nrows; j++) {
            std::vector<cv::KeyPoint> temp;
            int sx = i * cw, sy = j * ch;
            cv::Rect rect(sx, sy, cw, ch);
            cv::rectangle(image, rect, cv::Scalar(0, 0, 0));
            orb_feature_detector->detect(image(rect), temp);
            /* if (temp.size() < nfeatures / cells) { */
            /*     fallback_feature_detector->detect(image(rect), temp); */
            /* } */
            for (auto &p : temp) {
                keypoints->emplace_back(sx + p.pt.x, sy + p.pt.y, p.size, p.angle, p.response, p.octave, p.class_id);
            }
        }
    }
    orb_feature_detector->compute(image, *keypoints, *descriptors);
    printf("# Keypoints found in first image: %zu\n", keypoints->size());
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
    printf("# Keypoints found in first image: %zu\n", keypoints->size());
}

void match_features(std::vector<cv::KeyPoint> *keypoints1, std::vector<cv::KeyPoint> *keypoints2, cv::Mat *descriptors1, cv::Mat *descriptors2, std::vector<std::pair<int, int>> *matched) {
    cv::Ptr<cv::BFMatcher> bfmatcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> initialMatches;
    bfmatcher->knnMatch(*descriptors1, *descriptors2, initialMatches, 2);

    for (std::vector<cv::DMatch> m : initialMatches) {
        //lowes
        if (m[0].distance < m[1].distance * 0.7) {
            matched->push_back(std::make_pair(m[0].queryIdx, m[0].trainIdx));
        }
    }
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

    cv::Mat frame;
    std::vector<cv::KeyPoint> lastKeypoints;
    cv::Mat lastDescriptors;

    while (cap.isOpened()) {
        cap >> frame;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        /* extract_key_points(gray, &keypoints, &descriptors); */
        extract_key_points(frame, &keypoints, &descriptors, 5, 5);

        std::vector<std::pair<int, int>> matches;
        //match
        match_features(&lastKeypoints, &keypoints, &lastDescriptors, &descriptors, &matches);

        cv::Mat annotated = frame.clone();
        for (auto m : matches) {
            cv::line(annotated, lastKeypoints[m.first].pt, keypoints[m.second].pt, cv::Scalar(0,0, 255));
            /* cv::circle(annotated, p.pt, 2, cv::Scalar(0, 255, 0)); */
        }

        cv::imshow("points", annotated);
        if (cv::waitKey(16) == 113) {
            break;
        }

        lastKeypoints.swap(keypoints);
        lastDescriptors = descriptors;
    }
}
