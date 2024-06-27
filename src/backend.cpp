#include "slam/common.h"
#include "slam/backend.h"

namespace slam {

Backend::Backend() {
    
}

Backend::BundleAdjustment() {
    typedef g2o::LinearSolverCSparce<g2o::BlockSolver_6_3::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_6_3>(
            g2o::make_unique<LinearSolverType>()
        )
    );

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    std::unordered_map<unsigned long, vertexPose*> vertices;
    unsigned long max_kf_id = 0; 
    for (pose : poses) {
        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId();
        vertex_pose->setEstimate(pose);
        optimizer.addVertex(vertex_pose);
    }


}

}