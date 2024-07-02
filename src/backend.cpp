#include "backend.h"

namespace slam {

Backend::Backend() {
    
}

bool Backend::setCamera(const cv::Mat &k) {
    cv::cv2eigen(k, K);
    return true;
}

void Backend::BundleAdjustment(
    std::vector<Sophus::SE3d> &poses, 
    std::vector<std::vector<Eigen::Matrix<double, 3, 1>>> &positions,
    std::vector<std::vector<cv::Point2d>> &pixel_positions) {

    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(
        new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>()
    );
    std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr (new g2o::BlockSolver_6_3(std::move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    std::unordered_map<unsigned long, VertexSE3*> vertices;

    for (int i = 0; i < poses.size(); i++) {
        VertexSE3 *vertex_pose = new VertexSE3();
        vertex_pose->setId(i);
        vertex_pose->setEstimate(poses[i]);
        optimizer.addVertex(vertex_pose);
        vertices.insert({i, vertex_pose});
    }

    int index = 1;
    double chi = 5.991;
    std::unordered_map<unsigned long, VertexFeaturePos*> vertices_features;
    //std::unordered_map<EdgeProjection *, VertexSE3*> edges_features;
    
    for (int i = 0; i < positions.size(); i++) {
        for(int j = 0; j < positions[i].size(); j++) {
            VertexFeaturePos *vertex_feature = new VertexFeaturePos;
            vertex_feature->setEstimate(positions[i][j]);
            vertex_feature->setId(poses.size() + j);
            vertex_feature->setMarginalized(true);
            vertices_features.insert({poses.size() + j, vertex_feature});

            EdgeProjection *edge = new EdgeProjection(K, poses[i]);
            edge->setId(j);
            edge->setVertex(0, vertices.at(i));
            edge->setVertex(1, vertices_features.at(j));
            edge->setMeasurement(Eigen::Matrix<double, 2, 1>(pixel_positions[i][j].x, pixel_positions[i][j].y)); //set toVec2
            edge->setInformation(Eigen::Matrix<double, 2, 2>::Identity());
            
            auto rk = new g2o::RobustKernelHuber();
            rk->setDelta(chi);
            edge->setRobustKernel(rk);

            //edges_features.insert({edge, });
            optimizer.addEdge(edge);
        }
    }
    
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    delete solver;
    solver_ptr.release();
    linearSolver.release();
}

}