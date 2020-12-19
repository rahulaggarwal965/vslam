#include "../src/KDTree.cpp"
#include <queue>

//TODO: write failed cases into a file

void print_vector(const std::vector<cv::Point2f> &vec) {
    printf("[");
    for (int i = 0; i < vec.size() - 1; i++) {
        printf("{%f, %f}, ", vec[i].x, vec[i].y);
    }
    printf("{%f, %f}]\n", vec.back().x, vec.back().y);
}

// inefficient
void print_tree(const KDTree &kdtree) {
    std::queue<KDTree::KDTreeNode *> q;
    if (kdtree.root != NULL) {
        q.push(kdtree.root);
    }
    int i = 0, b = 0;
    while (!q.empty()) {
        const KDTree::KDTreeNode *t = q.front();
        q.pop();
        if (t == NULL) {
            printf("               ");
        } else {
            printf("{%3.2f, %3.2f} ", t->pt.x, t->pt.y);
            q.push(t->left);
            q.push(t->right);
        }
        if (i == b || q.empty()) {
            printf("\n");
            b = b * 2 + 2;
        }
        i++;
    }
}

void generate_random_point_array(std::vector<cv::Point2f> &arr, const int size) {
    arr.reserve(size);
    for (int j = 0; j < size; j++) {
        cv::Point2f pt(rand() % 100, rand() % 100);
        arr.emplace_back(rand() % 100, rand() % 100);
    }
}

void test_nearest_neighbor(const int n_trials = 100, const int min_size = 5, const int max_size = 35) {
    int n_success = 0;
    std::vector<cv::Point2f> arr;
    for (int i = 0; i < n_trials; i++) {
        int size = rand() % (max_size - min_size) + min_size;
        generate_random_point_array(arr, size);

        KDTree kdtree;
        construct_kdtree(kdtree, arr);
        cv::Point2f qp = {static_cast<float>(rand() % 100), static_cast<float>(rand() % 100)};
        cv::Point2f nn = nearest(kdtree, qp);

        printf("Testing array of size %d with query point {%3.2f, %3.2f}\n", size, qp.x, qp.y);

        // Find actual point
        float best_distance_sq = INFINITY;
        cv::Point2f nn_actual;
        for (cv::Point2f &pt : arr) {
            cv::Point diff = qp - pt;
            float curr_distance_sq = diff.dot(diff);
            if (curr_distance_sq < best_distance_sq) {
                best_distance_sq = curr_distance_sq;
                nn_actual = pt;
            }
        }
        if (nn == nn_actual) {
            n_success++;
        } else {
            cv::Point diff = qp - nn;
            float dist_sq = diff.dot(diff);
            if (dist_sq == best_distance_sq) {
                n_success++;
            } else {
                printf("\nFailure with nearest neigbor:\n");
                printf("Height: %d\n", kdtree.height);
                print_tree(kdtree);
                printf("Found nearest neighbor to {%3.2f, %3.2f}: {%3.2f, "
                       "%3.2f} and it was actually {%3.2f, %3.2f}\n\n",
                       qp.x, qp.y, nn.x, nn.y, nn_actual.x, nn_actual.y);
            }
        }
        free(kdtree.root);
        arr.clear();
    }
    printf("%d successes out of %d trials\n", n_success, n_trials);
}

void test_radius_search(const int n_trials = 100, const float min_radius = 10, const float max_radius = INFINITY, const int min_size = 5, const int max_size = 35) {
    int n_success = 0;
    std::vector<cv::Point2f> arr;
    for (int i = 0; i < n_trials; i++) {
        int size = rand() % (max_size - min_size) + min_size;
        float radius = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX) / (max_radius - min_radius)) + min_radius;
        float radius_sq = SQ(radius);
        generate_random_point_array(arr, size);

        KDTree kdtree;
        construct_kdtree(kdtree, arr);
        cv::Point2f qp = {static_cast<float>(rand() % 100), static_cast<float>(rand() % 100)};

        std::vector<cv::Point2f> found_pts = radius_search(kdtree, qp, radius);

        printf("Test --- Array size: %d - Query Point: {%3.2f, %3.2f} - Radius: %f\n", size, qp.x, qp.y, radius);

        std::vector<cv::Point2f> pts;
        // Find actual points
        for (cv::Point2f &pt : arr) {
            cv::Point diff = qp - pt;
            if (diff.dot(diff) < radius_sq) {
                pts.push_back(pt);
            }
        }
        bool fail = (found_pts.size() != pts.size());
        if (!fail) {
            const auto cmp = [](const cv::Point2f &p1, const cv::Point2f &p2) -> bool { return (p1.x == p2.x) ? p1.y < p2.y : p1.x < p2.x; };
            std::sort(found_pts.begin(), found_pts.end(), cmp);
            std::sort(pts.begin(), pts.end(), cmp);
            for (size_t i = 0; i < found_pts.size(); i++) {
                if (found_pts[i] != pts[i]) {
                    fail = true;
                }
            }
        }
        if (fail) {
            printf("Failure in radius_search\n");
            printf("Height: %d\n", kdtree.height);
            print_tree(kdtree);
            printf("Found points:\n");
            print_vector(found_pts);
            printf("Actual points:\n");
            print_vector(pts);
            printf("\n");
        } else {
            n_success++;
        }
        free(kdtree.root);
        arr.clear();
    }
    printf("%d successes out of %d trials\n", n_success, n_trials);
}

int main(void) {
    test_nearest_neighbor(1000, 2500, 3000);
    test_radius_search(1000, 10, 100, 2500, 3000);
}
