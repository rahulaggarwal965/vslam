#include <cstdio>
#include <cstdlib>
#include <vector>
#include "time.h"

struct Point {
    int x;
    int y;
};

struct Node {
    Point p;
    Point *left, *right;
};

Point * find_median(std::vector<Point*> points, int axis, int * comparison_count) {
    if (axis == 0) {

        //TODO: better way of finding pivot
        int n = rand() % points.size();
        Point *temp = points[n];
        points[n] = points[0];
        points[0] = temp;
        int m = (*temp).x;
        int i = 1;
        int j = points.size() - 1;
        while (i <= j) {
            while (i <= j && (*points[i]).x <= m) {
                (*comparison_count)++;
                i++;
            }
            while (i <= j && (*points[j]).x >= m) {
                (*comparison_count)++;
                j--;
            }
            if (i <= j) {
                Point *temp = points[i];
                points[i++] = points[j];
                points[j--] = temp;
            }
        }
        int lsize = j + 1;
        int rsize = points.size() - lsize;
        if (points.size() / 2 < lsize) {
            int l = 0;
            int r = j;
        } else {
            int l = j + 1;
            int r = points.size();
        }
    }
}


void quick_sort(std::vector<Point*> &points, int l, int r, int axis, int *comparison_count) {
    //TODO: better way of finding pivot
    int n = rand() % (r - l + 1) + l;
    Point *temp = points[n];
    points[n] = points[l];
    points[l] = temp;
    int m = (axis == 0) ? (*temp).x : (*temp).y;
    int i = l + 1;
    int j = r;
    while (i <= j) {
        while (i <= j && (*points[i]).x <= m) {
            (*comparison_count)++;
            i++;
        }
        while (i <= j && (*points[j]).x >= m) {
            (*comparison_count)++;
            j--;
        }
        if (i <= j) {
            Point *t = points[i];
            points[i++] = points[j];
            points[j--] = t;
        }
    }
    points[l] = points[j];
    points[j] = temp;
    quick_sort(points, l, j, axis, comparison_count);
    quick_sort(points, j + 1, r, axis, comparison_count);
}

void quick_sort(std::vector<Point*> &points, int axis, int *comparison_count) {
    quick_sort(points, 0, points.size() - 1, axis, comparison_count);
}

void create_kdtree(std::vector<Point> points, Node *node, int axis) {


    /* for () */

}

int main(void) {
    srand(time(0));
    std::vector<Point *> points;
    points.resize(10);
    for (int i = 0; i < points.size(); i++) {
        points[i] = (Point *) malloc(sizeof(Point));
        points[i]->x = rand() % 100;
        points[i]->y = 0;
        printf("[%d]: (%d, %d)\n", i, points[i]->x, points[i]->y);
    }
    int cc;
    quick_sort(points, 0, &cc);

    for (auto &p : points) {
        free(p);
    }
}


