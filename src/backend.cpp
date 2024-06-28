#include "slam/backend.h"

namespace slam {

Backend::Backend() {
    
}

Backend::BundleAdjustment(
    const std::vector<Sophus::SE3d>> &poses, 
    const std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> &positions) {
    typedef g2o::LinearSolverCSparce<g2o::BlockSolver_6_3::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_6_3>(
            g2o::make_unique<LinearSolverType>()
        )
    );

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    std::unordered_map<unsigned long, VertexSE3*> vertices;

    for (int i = 0; i < poses.size(); i++) {
        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(i);
        vertex_pose->setEstimate(poses[i]);
        optimizer.addVertex(vertex_pose);
        verticies.insert({i, vertex_pose});
    }

    int index = 1;
    double chi = 5.991;
    std::unordered_map<unsigned long, VertexFeaturePos*> vertices_features;
    
    for (int i = 0; i < positions.size(); i++) {
        for(int j = 0; j < positions[i].size(); j++) {
            VertexFeaturePos *vertex_feature = new VertexFeaturePos;
            vertex_feature->setEstimate(positions[i][j]);
            vertex_feature->setId(poses.size() + j);
            vertex_feature->setMarginalized(true);
            vertices_features.insert({poses.size() + j, vertex_feature});
        }
    }

}

virtual void VertexSE3::setToOriginImpl() override {
    _estimate = Sophus::SE3();
}

virtual void VertexSE3::oplusImpl(const double *update) override {
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3::exp(update_eigen) * _estimate;        
}

virtual void VertexFeaturePos::setToOriginImpl() override {
    _estimate = Eigen::Matrix<double, 3, 1>;
}

virtual void VertexFeaturePos::oplusImpl(const double *update) override {
    _estimate[0] = update[0];
    _estimate[1] = update[1];
    _estimate[2] = update[2];
}

}