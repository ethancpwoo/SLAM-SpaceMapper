#pragma once
#ifndef BACKEND_H
#define BACKEND_H

#include "common.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>


namespace slam {

class VertexSE3 : public g2o::BaseVertex<6, Sophus::SE3d> {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
        virtual void setToOriginImpl() override {
            _estimate = Sophus::SE3d();
        }

        virtual void oplusImpl(const double *update) override {
            Eigen::Matrix<double, 6, 1> update_eigen;
            update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
            _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;        
        }

        virtual bool read(std::istream &in) override { return true; }
        virtual bool write(std::ostream &out) const override { return true; }

};

class VertexFeaturePos : public g2o::BaseVertex<3, Eigen::Matrix<double, 3, 1>> {
    
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
        virtual void setToOriginImpl() override {
            _estimate = Eigen::Matrix<double, 3, 1>::Zero();
        }

        virtual void oplusImpl(const double *update) override {
            _estimate[0] = update[0];
            _estimate[1] = update[1];
            _estimate[2] = update[2];
        }

        virtual bool read(std::istream &in) override {return true;}
        virtual bool write(std::ostream &out) const override {return true;}

};

class EdgeProjection : public g2o::BaseBinaryEdge<2, Eigen::Matrix<double, 2, 1>, VertexSE3, VertexFeaturePos> {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
        EdgeProjection(const Eigen::Matrix<double, 3, 3> &k, const Sophus::SE3d &cam_ext) : _k(k){
            _cam_ext = cam_ext;
        }
        
        virtual void computeError() override {
            const VertexSE3 *v0 = static_cast<VertexSE3 *>(_vertices[0]);
            const VertexFeaturePos *v1 = static_cast<VertexFeaturePos *>(_vertices[1]);
            Sophus::SE3d T = v0->estimate();
            Eigen::Matrix<double, 3, 1> pos_pixel = _k * (_cam_ext * (T * v1->estimate()));
            pos_pixel /= pos_pixel[2];
            _error = _measurement - pos_pixel.head<2>();
        }

        virtual void linearizeOplus() override {
            const VertexSE3 *v0 = static_cast<VertexSE3 *>(_vertices[0]);
            const VertexFeaturePos *v1 = static_cast<VertexFeaturePos *>(_vertices[1]);
            Sophus::SE3d T = v0->estimate();
            Eigen::Matrix<double, 3, 1> pos_world = v1->estimate();
            Eigen::Matrix<double, 3, 1> pos_cam = _cam_ext * T * pos_world;
            // define intrisics
            double fx = _k(0, 0);
            double fy = _k(1, 1);
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double Zinv = 1.0 / (Z + 1e-18); 
            double Zinv2 = Zinv * Zinv;

            _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
                fy * Y * Zinv2, fy + fy * Y * Y, -fy * X * Y * Zinv2,
                -fy * X * Zinv;
            
            _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) * _cam_ext.rotationMatrix() * T.rotationMatrix();
        }

        virtual bool read(std::istream &in) override {return true;}
        virtual bool write(std::ostream &out) const override {return true;}

    private:
        Eigen::Matrix<double, 3, 3> _k;
        Sophus::SE3d _cam_ext;

};

class Backend {

    public:
        Backend();
        void BundleAdjustment(
            std::deque<Sophus::SE3d> &poses, 
            std::deque<std::vector<cv::Point3d>> &positions,
            std::deque<std::vector<cv::Point2d>> &pixel_positions
        );
        bool setCamera(const cv::Mat &k);
    private:
        Eigen::Matrix<double, 3, 3> K;
        // pos camera SE3

};
    
}

#endif