#include "slam/common.h"
#include "slam/backend.h"

namespace slam {

Backend::Backend() {

}

Backend::BundleAdjustment() {
    std::map<unsigned long, g2o::BaseVertex<6, Sophus::SE3d>> vertices;
    for (auto &keyframe, keyframe) {
        g2o::BaseVertex<6, g2o::SE3> *vertex = new g2o::BaseVertex<6, g2o::SE3>();
        vertex->setId();
        vertex->setEsimate();
        optimizer.addVertex();

        vertices.insert();
    }

    std::map<unsigned long, g2o::BaseVertex<3, Eigen::Matrix<double, 3, 1>>> landmarks;

    // Get Intrinsics and Pose for camera

    int index = 1;
    double chi2_th = 5.991; // robust kernel
    std::map;
    
    // edges

    for(auto &landmark : landmarks) {
        auto observations; 
        for (auto &obs: observations) {
            auto feat = obs.lock();
            if (vertice
        }
    }



}

}