#include "KDTree.h"

KDTree::KDTreeNode *construct_kdtree(KDTree &kdtree, std::vector<cv::Point2f> &points,
                                     const std::vector<cv::Point2f>::iterator l,
                                     const std::vector<cv::Point2f>::iterator r, int axis) {
    if (l < r) {
        const size_t len = r - l;
        auto m = l + len / 2;
        if (axis == 0) {
            std::nth_element(l, m, r, [](const cv::Point2f &p1, const cv::Point2f &p2) -> bool { return p1.x < p2.x; });
        } else {
            std::nth_element(l, m, r, [](const cv::Point2f &p1, const cv::Point2f &p2) -> bool { return p1.y < p2.y; });
        }
        /* printf("Adding at index(%zu): (%f, %f)\n", kdtree.size, m->x, m->y);
         */
        KDTree::KDTreeNode &node = kdtree.root[kdtree.size++];
        node.pt = *m;
        node.left = construct_kdtree(kdtree, points, l, m, 1 - axis);
        node.right = construct_kdtree(kdtree, points, m + 1, r, 1 - axis);
        return &node;
    }
    return NULL;
}

void construct_kdtree(KDTree &kdtree, const std::vector<cv::Point2f> &points) {
    int N = points.size();
    if (N == 0) {
        kdtree.root = NULL;
    } else {
        kdtree.root = (KDTree::KDTreeNode *)malloc(N * sizeof(KDTree::KDTreeNode));
        std::vector<cv::Point2f> points_copy = points;
        construct_kdtree(kdtree, points_copy, points_copy.begin(), points_copy.end(), 0);
        kdtree.height = floor(log2(N)) + 1;
    }
}

cv::Point2f nearest(const KDTree &kdtree, const cv::Point2f &query_point, float max_distance_sq) {
    // TODO(rahul): bug where return is just {0, 0} when max_distance_sq is
    // small (return optional/struct?)
    cv::Point2f r;
    nearest(kdtree.root, query_point, 0, &r, &max_distance_sq);
    return r;
}

void nearest(KDTree::KDTreeNode *node, const cv::Point2f &query_pt, int axis, cv::Point2f *best_pt,
             float *best_distance_sq) {
    if (node == NULL) {
        return;
    }
    const cv::Point2f &pt = node->pt;
    const float split_distance = P(query_pt, axis) - P(pt, axis);

    KDTree::KDTreeNode *opposite_subtree;
    if (split_distance < 0) {
        nearest(node->left, query_pt, 1 - axis, best_pt, best_distance_sq);
        opposite_subtree = node->right;
    } else {
        nearest(node->right, query_pt, 1 - axis, best_pt, best_distance_sq);
        opposite_subtree = node->left;
    }

    cv::Point2f d = pt - query_pt;
    float curr_distance_sq = d.dot(d);
    if (curr_distance_sq < *best_distance_sq) {
        *best_distance_sq = curr_distance_sq;
        *best_pt = pt;
    }
    if (SQ(split_distance) < *best_distance_sq) {
        nearest(opposite_subtree, query_pt, 1 - axis, best_pt, best_distance_sq);
    }
}

std::vector<cv::Point2f> radius_search(const KDTree &kdtree, const cv::Point2f &query_pt, float radius) {
    float radius_sq = SQ(radius);
    std::vector<cv::Point2f> pts;
    radius_search(kdtree.root, query_pt, pts, radius, radius_sq, 0);
    return pts;
}

void radius_search(KDTree::KDTreeNode *node, const cv::Point2f &query_pt, std::vector<cv::Point2f> &pts, float radius,
                   float radius_sq, int axis) {
    if (node == NULL) {
        return;
    }
    const cv::Point2f pt = node->pt;
    const float split_distance = P(query_pt, axis) - P(pt, axis);

    if (ABS(split_distance) <= radius) {
        cv::Point2f diff = query_pt - pt;
        float dist_sq = diff.dot(diff);
        if (dist_sq < radius_sq) {
            pts.push_back(pt);
        }
        radius_search(node->left, query_pt, pts, radius, radius_sq, 1 - axis);
        radius_search(node->right, query_pt, pts, radius, radius_sq, 1 - axis);
    } else if (split_distance < 0) {
        radius_search(node->left, query_pt, pts, radius, radius_sq, 1 - axis);
    } else {
        radius_search(node->right, query_pt, pts, radius, radius_sq, 1 - axis);
    }
}

/*
    FRAME-specific kdtree
   */

void construct_kdtree(frame_kdtree &kdtree, const std::vector<cv::Point2f> &points) {
    usize N = points.size();
    if (N == 0) {
        kdtree.root = NULL;
    } else {
        kdtree.root = (frame_kdtree::KDTreeNode *)malloc(N * sizeof(frame_kdtree::KDTreeNode));
        std::vector<usize> point_indices;
        point_indices.reserve(points.size());
        for (usize i = 0; i < N; i++) {
            point_indices.push_back(i);
        }
        construct_kdtree(kdtree, points, point_indices, point_indices.begin(), point_indices.end(), 0);
        kdtree.height = floor(log2(N)) + 1;
    }
}
frame_kdtree::KDTreeNode *construct_kdtree(frame_kdtree &kdtree, const std::vector<cv::Point2f> &points, std::vector<usize> &point_indices,
                                           const std::vector<usize>::iterator l,
                                           const std::vector<usize>::iterator r, int axis) {
    if (l < r) {
        const size_t len = r - l;
        auto m = l + len / 2;
        std::nth_element(l, m, r, [points, axis](const usize &i1, const usize &i2) -> bool {return P(points[i1], axis) < P(points[i2], axis); });
        /* if (axis == 0) { */
        /*     std::nth_element(l, m, r, [points](const usize &i1, const usize &i2) -> bool { return points[i1].x < points[i2].x; }); */
        /* } else { */
        /*     std::nth_element(l, m, r, [](const cv::Point2f &p1, const cv::Point2f &p2) -> bool { return p1.y < p2.y; }); */
        /* } */
        /* printf("Adding at index(%zu): (%f, %f)\n", kdtree.size, m->x, m->y);
         */
        frame_kdtree::KDTreeNode &node = kdtree.root[kdtree.size++];
        node.pt_index = *m;
        node.left  = construct_kdtree(kdtree, points, point_indices, l, m, 1 - axis);
        node.right = construct_kdtree(kdtree, points, point_indices, m + 1, r, 1 - axis);
        return &node;
    }
    return NULL;
}

std::vector<usize> radius_search(const frame_kdtree kdtree, const std::vector<cv::Point2f> &points, const cv::Point2f &query_pt, float radius) {
    float radius_sq = SQ(radius);
    std::vector<usize> indices;
    radius_search(kdtree.root, points, query_pt, indices, radius, radius_sq, 0);
    return indices;
}
void radius_search(frame_kdtree::KDTreeNode *node, const std::vector<cv::Point2f> &points, const cv::Point2f &query_pt, std::vector<usize> &indices, float radius, float radius_sq, int axis) {
    if (node == NULL) {
        return;
    }
    const cv::Point2f &pt = points[node->pt_index];
    const float split_distance = P(query_pt, axis) - P(pt, axis);

    if (ABS(split_distance) <= radius) {
        cv::Point2f diff = query_pt - pt;
        float dist_sq = diff.dot(diff);
        if (dist_sq < radius_sq) {
            indices.push_back(node->pt_index);
        }
        radius_search(node->left, points, query_pt, indices, radius, radius_sq, 1 - axis);
        radius_search(node->right, points, query_pt, indices, radius, radius_sq, 1 - axis);
    } else if (split_distance < 0) {
        radius_search(node->left, points, query_pt, indices, radius, radius_sq, 1 - axis);
    } else {
        radius_search(node->right, points, query_pt, indices, radius, radius_sq, 1 - axis);
    }
}
