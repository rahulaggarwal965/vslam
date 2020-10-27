#include <opencv2/core.hpp>
#include <pangolin.h>
#include <thread>

class Display {

    public:
        Display(const char *window_name, const int W, const int H);

        void initialize();
        void run();

        void draw_points(std::vector<cv::Point3f> points);
        void draw_boxes(std::vector<cv::Mat*> boxes);

    private:
        const char *window_name;
        const int W, H;
};
