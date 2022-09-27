#include "Display.h"
#include <mutex>

Display::Display(const char *window_name, int W, int H, std::mutex *mtx)
    : mtx(mtx),
    window_name(window_name),
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
        mtx->lock();
        /* while (!q.empty()) { */
        /*     ds = q.front(); */
        /*     q.pop(); */
        /* } */
        if (ds.points != NULL) {
            printf("Points size: %zu\n", ds.size);
            /* glColor3f(1.0, 0.0, 0.0); */
            /* draw_points(*ds.points, ds.size); */
            printf("Colors size: %zu\n", ds.colors->size());
            draw_points_colors(*ds.points, *ds.colors, ds.size);
        }
        if (ds.frames != NULL) {
            glColor3f(0.0, 0.0, 1.0);
            for (auto &f : *ds.frames) {
                draw_box(f.pose);
            }
        }
        mtx->unlock();

        pangolin::FinishFrame();
    }

    pangolin::GetBoundWindow()->RemoveCurrent();
}

void Display::close() {
    pangolin::Quit();
}

void Display::join() {
    loop.join();
}

void Display::draw_points(const std::vector<cv::Point3f> &points) {
    glBegin(GL_POINTS);
    for (size_t i = 0; i < points.size(); i++) {
        glVertex3d(points[i].x, points[i].y, points[i].z);
    }
    glEnd();
}

void Display::draw_points(const cv::Mat &points, const usize size) {
    glBegin(GL_POINTS);
    if (!points.isContinuous()) {
        fprintf(stderr, "Error: not continuous in %s, line %d\n", __FILE__, __LINE__);
        return;
    }
    // NOTE(rahul): expect points to be Nx4 and continuous ...
    /* float *data = (float *) points.data; */
    const float *data = points.ptr<float>(0);
    for (usize i = 0; i < size; i += 4) {
        glVertex3d(data[i], data[i + 1], data[i + 2]);
        /* glVertex3d(points[i].x, points[i].y, points[i].z); */
    }
    glEnd();
}

void Display::draw_points_colors(const cv::Mat &points, const std::vector<cv::Point3_<u8>> &colors, const usize size) {
    glBegin(GL_POINTS);
    if (!points.isContinuous()) {
        fprintf(stderr, "Error: not continuous in %s, line %d\n", __FILE__, __LINE__);
        return;
    }
    // NOTE(rahul): expect points to be Nx4 and continuous ...
    /* float *data = (float *) points.data; */
    const f32 *data = points.ptr<f32>();
    for (usize i = 0, j = 0; i < size; i += 4, j++) {
        if (j >= colors.size()) {
            /* fprintf(stderr, "Accessed color index %zu out of color size %zu\n", j, colors.size()); */
        } else {
            glColor3b(colors[j].x, colors[j].y, colors[j].z);
        }
        glVertex3d(data[i], data[i + 1], data[i + 2]);
        /* glVertex3d(points[i].x, points[i].y, points[i].z); */
    }
    glEnd();
}

void Display::draw_box(const cv::Mat &pose, float w, float h_ratio, float z_ratio) {
    float h = w * h_ratio;
    float z = w * z_ratio;

    glPushMatrix();
    if (!pose.isContinuous()) {
        return;
    }
    glMultTransposeMatrixf(pose.ptr<float>(0));

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

void Display::draw_boxes(std::vector<cv::Mat*> boxes, float w, float h_ratio, float z_ratio) {
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
