#ifndef LOOPCLOSE_H
#define LOOPCLOSE_H

#include <fbow/fbow.h>

#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>

#include "common.h"

namespace slam {

class LoopClose {

    public:
        LoopClose();
        int findLoop(const cv::Mat &current_descriptor);
        bool setCamera(const cv::Mat &k);
        void optimize(const Sophus::SE3d &loop, Sophus::SE3d &map);

    private:
        g2o::SparseOptimizer optimizer;
        fbow::Vocabulary vocab;
        std::vector<fbow::fBow> prev_bows;
        Eigen::Matrix<double, 3, 3> K;

};

}

#endif