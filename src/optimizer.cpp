
#include "Frame.h"
#include <vector>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>

void optimize(std::vector<Frame*>& frames) {
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
}
