#include "map.h"

namespace slam {

    Map::Map() {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        Sophus::SE3d p(mat);
        // info.push_back(p);
    }

    void Map::insertKeyPoint(const Sophus::SE3d pose, const std::vector<Eigen::Vector3d> feature) {
        poses.push_back(pose);
        features.push_back(feature);
    }

    std::vector<Sophus::SE3d> Map::getMapPose(){
        return poses;
    }
    
    std::vector<std::vector<Eigen::Vector3d>> Map::getMapFeature() {
        return features;
    }

}