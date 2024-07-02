#include "backend.h"

namespace slam {

Backend::Backend() {
    
}

bool Backend::setCamera(const cv::Mat &k) {
    cv::cv2eigen(k, K);
    return true;
}

void Backend::BundleAdjustment(
    std::deque<Sophus::SE3d> &poses, 
    std::deque<std::vector<cv::Point3d>> &positions,
    std::deque<std::vector<cv::Point2d>> &pixel_positions) {

    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(
        new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>()
    );
    std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr (new g2o::BlockSolver_6_3(std::move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    std::unordered_map<unsigned long, VertexSE3*> vertices;

    std::cout << poses.size() << std::endl;
    std::cout << positions.size() << std::endl;
    std::cout << pixel_positions.size() << std::endl;

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
            // std::cout << positions[i].size() << std::endl;
            // std::cout << pixel_positions[i].size() << std::endl;
            VertexFeaturePos *vertex_feature = new VertexFeaturePos;
            vertex_feature->setEstimate(Eigen::Matrix<double, 3, 1>(positions[i][j].x, positions[i][j].y, positions[i][j].z));
            vertex_feature->setId(poses.size() + j);
            vertex_feature->setMarginalized(true);
            vertices_features.insert({j, vertex_feature});

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
    optimizer.optimize(10); //solver and other pointers automatically gets deleted

}

}