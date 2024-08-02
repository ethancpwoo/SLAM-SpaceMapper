#include "loopclose.h"
#include "backend.h"

namespace slam {

LoopClose::LoopClose() {
    vocab.readFromFile("../../test_data/orb_mur.fbow");
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(
        new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>()
    );
    std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr (new g2o::BlockSolver_6_3(std::move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));

    optimizer.setAlgorithm(solver);
}

bool LoopClose::setCamera(const cv::Mat &k) {
    cv::cv2eigen(k, K);
    return true;
}

int LoopClose::findLoop(const cv::Mat &current_descriptor) {
    int index_detected = -1;
    fbow::fBow current_bow = vocab.transform(current_descriptor);
    for(int i = 0; i < prev_bows.size(); i++) {
        std::cout << fbow::fBow::score(current_bow, prev_bows[i]) << std::endl;
        if (fbow::fBow::score(current_bow, prev_bows[i]) > 0.8) {
            index_detected = i;
            break;
        }
    }
    prev_bows.push_back(current_bow);
    return index_detected;
}

void LoopClose::optimize(const Sophus::SE3d &loop, Sophus::SE3d &map) {

    VertexSE3 *vertex_pose_loop = new VertexSE3();
    VertexSE3 *vertex_pose_map = new VertexSE3();
    vertex_pose_loop->setId(0);
    vertex_pose_map->setId(1);
    vertex_pose_loop->setEstimate(loop);
    vertex_pose_map->setEstimate(map);

    // they are suppoosed to be the same
    Eigen::Isometry3d measurement = Eigen::Isometry3d::Identity();
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    
    edge->setVertex(0, vertex_pose_loop);
    edge->setVertex(1, vertex_pose_map);
    edge->setMeasurement(measurement);
    edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity()); 

    // Set the information matrix (e.g., identity or based on your uncertainty estimation)
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
    edge->setInformation(information);

    optimizer.addEdge(edge);

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    VertexSE3 *vertex_pose = static_cast<VertexSE3*>(optimizer.vertex(1));
    map = vertex_pose->estimate();
}

}