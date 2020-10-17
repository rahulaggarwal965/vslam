#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "Slam.h"
#include "opencv2/videoio.hpp"

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Expected video file name as a command line argument\n");
        return -1;
    }

    const char * vName = argv[1];
    cv::VideoCapture cap(vName);
    int W = (int) cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int H = (int) cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int CNT = (int) cap.get(cv::CAP_PROP_FRAME_COUNT);

    //TODO: calibrate?
    float focal = (argc > 2) ? atof(argv[2]) : 525.0f;
    float data[9] = {focal, 0, static_cast<float>(W/2), 0, focal, static_cast<float>(H/2), 0, 0, 1};
    cv::Mat K(3, 3, CV_32FC1, data);

    Slam slam(W, H, K);

    int frame_cnt = 0;
    cv::Mat frame;
    while(cap.isOpened()) {
        cap >> frame;
        if(frame.empty()) {
            fprintf(stderr, "Could not get frame of video");
        }

        slam.process_frame(frame);

        /* cv::Mat drawn; */
        /* slam.point_map.frames[slam.point_map.frames.size() - 1]->draw(frame, drawn); */
        /* cv::imshow("2d", drawn); */

        frame_cnt++;

        /* if (cv::waitKey(0) == 113) { */
        /*     break; */
        /* } */
    }

}
