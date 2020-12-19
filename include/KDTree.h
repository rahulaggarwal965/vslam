#ifndef __KDTREE_H__
#define __KDTREE_H__

#include <algorithm>
#include <opencv2/core.hpp>

#include <vslam_internal.h>

#define SQ(x) ((x) * (x))
#define ABS(x) (((x) > 0) ? x : -x)
#define P(pt, i) ((float *)&(pt))[i]

struct KDTree {
    struct KDTreeNode {
        cv::Point2f pt;
        KDTreeNode *left;
        KDTreeNode *right;
    };

    KDTreeNode *root;
    u32 size = 0;
    u8 height = 0;
};

void construct_kdtree(KDTree &kdtree, const std::vector<cv::Point2f> &points);
KDTree::KDTreeNode *construct_kdtree(KDTree &kdtree, std::vector<cv::Point2f> &points,
                                     const std::vector<cv::Point2f>::iterator l,
                                     const std::vector<cv::Point2f>::iterator r, int axis);

cv::Point2f nearest(const KDTree &kdtree, const cv::Point2f &query_pt, float max_distance_sq = INFINITY);
void nearest(KDTree::KDTreeNode *node, const cv::Point2f &query_pt, int axis, cv::Point2f *best_pt,
             float *best_distance_sq);

cv::Point2f nearest_approx(const KDTree &kdtree, const cv::Point2f &query_pt, int max_nodes,
                           float max_distance_sq = INFINITY);
void nearest_approx(KDTree::KDTreeNode *node, const cv::Point2f &query_pt, int max_nodes, int axis,
                    cv::Point2f *best_pt, float *best_distance_sq);

/* cv::Point2f k_nearest(const KDTree &kdtree, const cv::Point2f &query_point, int k, float max_distance_sq = INFINITY);
 */
/* void k_nearest(KDTree::KDTreeNode *node, const cv::Point2f &query_pt, int k, int axis, cv::Point2f *best_pt, */
/*                float *best_distance_sq); */

std::vector<cv::Point2f> radius_search(const KDTree &kdtree, const cv::Point2f &query_pt, float radius);
void radius_search(KDTree::KDTreeNode *node, const cv::Point2f &query_pt, std::vector<cv::Point2f> &pts, float radius, float radius_sq, int axis);

struct frame_kdtree {
    struct KDTreeNode {
        memory_index pt_index;
        KDTreeNode *left;
        KDTreeNode *right;
    };

    KDTreeNode *root;
    u32 size = 0;
    u8 height = 0;
};


void construct_kdtree(frame_kdtree &kdtree, const std::vector<cv::Point2f> &points);
frame_kdtree::KDTreeNode *construct_kdtree(frame_kdtree &kdtree, const std::vector<cv::Point2f> &points, std::vector<memory_index> &point_indices,
                                     const std::vector<memory_index>::iterator l,
                                     const std::vector<memory_index>::iterator r, int axis);

cv::Point2f nearest(const frame_kdtree &kdtree, const cv::Point2f &query_pt, float max_distance_sq = INFINITY);
void nearest(frame_kdtree::KDTreeNode *node, const cv::Point2f &query_pt, int axis, cv::Point2f *best_pt,
             float *best_distance_sq);

cv::Point2f nearest_approx(const frame_kdtree &kdtree, const cv::Point2f &query_pt, int max_nodes,
                           float max_distance_sq = INFINITY);
void nearest_approx(frame_kdtree::KDTreeNode *node, const cv::Point2f &query_pt, int max_nodes, int axis,
                    cv::Point2f *best_pt, float *best_distance_sq);

/* cv::Point2f k_nearest(const KDTree &kdtree, const cv::Point2f &query_point, int k, float max_distance_sq = INFINITY);
 */
/* void k_nearest(frame_kdtree::KDTreeNode *node, const cv::Point2f &query_pt, int k, int axis, cv::Point2f *best_pt, */
/*                float *best_distance_sq); */

std::vector<memory_index> radius_search(const frame_kdtree kdtree, const std::vector<cv::Point2f> &points, const cv::Point2f &query_pt, float radius);
void radius_search(frame_kdtree::KDTreeNode *node, const std::vector<cv::Point2f> &points, const cv::Point2f &query_pt, std::vector<memory_index> &indices, float radius, float radius_sq, int axis);


#endif
