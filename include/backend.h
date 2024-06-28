#pragma once
#ifndef BACKEND_H
#define BACKEND_H

#include "common.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>


namespace slam {

class VertexSE3 : public g2o::BaseVertex<6, Sophus::SE3> {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        virtual void setToOriginImpl() override;
        virtual void oplusImpl(const double *update) override;
        virtual bool read(std::istream &in) override { return true; }
        virtual bool write(std::ostream &out) const override { return true; }

};

class VertexFeaturePos : public g2o::BaseVertex<3, Eigen::Matrix<double, 3, 1>> {
    
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        virtual void setToOriginImpl() override; 
        virtual void oplusImpl(const double *update) override;
        virtual bool read(std::istream &in) override {return true};
        virtual bool write(std::ostream &out) override {return true};

};

class Backend {

    public:
        Backend();
        void BundleAdjustment();

    private:
        

};
    
}

#endif