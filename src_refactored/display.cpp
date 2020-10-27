#include "Display.h"

Display::Display(const char *window_name, int W, int H)
    : window_name(window_name),
    W(W),
    H(H)
{}

void Display::initialize() {

    pangolin::CreateWindowAndBind(window_name, W, H);
    glEnable(GL_DEPTH_TEST);
    pangolin::GetBoundWindow()->RemoveCurrent();

    this->loop = std::thread(&Display::run, this);
}

void Display::run() {
    pangolin::BindToContext(this->window_name);
    glEnable(GL_DEPTH_TEST);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(W, H, 420, 420, W / 2, H / 2, 0.2, 10000),
            pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY)
            );

    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, (double) -W/H)
        .SetHandler(&handler);

    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        //TODO: implement queue
        pangolin::glDrawColouredCube();

        pangolin::FinishFrame();
    }

    pangolin::GetBoundWindow()->RemoveCurrent();
}

void Display::join() {
    loop.join();
}

void draw_points(std::vector<cv::Point3f> points) {
    glBegin(GL_POINTS);
    for (size_t i = 0; i < points.size(); i++) {
        glVertex3d(points[i].x, points[i].y, points[i].z);
    }
    glEnd();
}

void draw_boxes(std::vector<cv::Mat*> boxes, float w=1.0, float h_ratio=0.75, float z_ratio=0.6) {
    float h = w * h_ratio;
    float z = w * z_ratio;

    for (size_t i = 0; i < boxes.size(); i++) {
        glPushMatrix();
        glMultTransposeMatrixf(boxes[i]->ptr<float>(0));

        glBegin(GL_LINES);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }
}
