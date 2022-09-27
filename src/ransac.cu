#include "vslam_internal.h"
#include <vector>

struct model {

};

template <typename T, typename M, int block_size = >
__global__
static void ransac(int *ransac_sets, int set_size, T *data, M *model) {

    const int bx = blockIdx.x;
    __shared__ f32 residuals[blockDim.x]:
    auto set = ransac_sets + threadIdx.x * set_size;
    typename M::out r = model->compute_model(set);

    f32 residual = FLT_MAX;
    residuals[threadIdx.x] = residual = model->compute_residual(M::out, set);

    for (int i = blockDim.x / 2; i > 16; i >>= 1) {
        if (i < blockDim.x / 2) {
            residuals[]
        }
    }

} 
