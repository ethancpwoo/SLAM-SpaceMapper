#pragma once
#ifndef BACKEND_H
#define BACKEND_H

#include "common.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
// #include <g2o/core/robust_kernel.h>
// #include <g2o/core/sparse_optimizer.h>
// #include <g2o/solvers/eigen/linear_solver_eigen.h>


namespace slam {

// class VertexSE3 : public g2o::BaseVertex<6, Sophus::SE3d> {

//     public:
//         EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
//         virtual void setToOriginImpl() override {
//             _estimate = Sophus::SE3d();
//         }

//         virtual void oplusImpl(const double *update) override {
//             Eigen::Matrix<double, 6, 1> update_eigen;
//             update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
//             _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;        
//         }

//         virtual bool read(std::istream &in) override { return true; }
//         virtual bool write(std::ostream &out) const override { return true; }

// };

// class VertexFeaturePos : public g2o::BaseVertex<3, Eigen::Matrix<double, 3, 1>> {
    
//     public:
//         EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
//         virtual void setToOriginImpl() override {
//             _estimate = Eigen::Matrix<double, 3, 1>::Zero();
//         }

//         virtual void oplusImpl(const double *update) override {
//             _estimate[0] = update[0];
//             _estimate[1] = update[1];
//             _estimate[2] = update[2];
//         }

//         virtual bool read(std::istream &in) override {return true;}
//         virtual bool write(std::ostream &out) const override {return true;}

// };

class Backend {

    public:
        Backend();
        void BundleAdjustment(
            std::vector<Sophus::SE3d> &poses, 
            std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> &positions
        );

};
    
}

#endif