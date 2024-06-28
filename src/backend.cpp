#include "backend.h"

namespace slam {

Backend::Backend() {
    
}

void Backend::BundleAdjustment(
    std::vector<Sophus::SE3d> &poses, 
    std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> &positions) {

    // std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver(
    //     new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>()
    // );
    // std::unique_ptr<g2o::BlockSolverX> solver_ptr (new g2o::BlockSolverX(std::move(linearSolver)));
    // g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));

    // g2o::SparseOptimizer optimizer;
    // optimizer.setAlgorithm(solver);
    // optimizer.setVerbose(true);

    // std::unordered_map<unsigned long, VertexSE3*> vertices;

    // for (int i = 0; i < poses.size(); i++) {
    //     VertexSE3 *vertex_pose = new VertexSE3();
    //     vertex_pose->setId(i);
    //     vertex_pose->setEstimate(poses[i]);
    //     optimizer.addVertex(vertex_pose);
    //     vertices.insert({i, vertex_pose});
    // }

    // int index = 1;
    // double chi = 5.991;
    // std::unordered_map<unsigned long, VertexFeaturePos*> vertices_features;
    
    // for (int i = 0; i < positions.size(); i++) {
    //     for(int j = 0; j < positions[i].size(); j++) {
    //         VertexFeaturePos *vertex_feature = new VertexFeaturePos;
    //         vertex_feature->setEstimate(positions[i][j]);
    //         vertex_feature->setId(poses.size() + j);
    //         vertex_feature->setMarginalized(true);
    //         vertices_features.insert({poses.size() + j, vertex_feature});
    //     }
    // }
    
    // delete solver;
    // solver_ptr.release();
    // linearSolver.release();

}

}