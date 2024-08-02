#include "map.h"

namespace slam {

    Map::Map() {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        global_pos = Sophus::SE3d(mat);
        // info.push_back(p);
    }

    void Map::insertKeyPoint(Sophus::SE3d &pose, std::vector<cv::Point3d> &feature) {
        poses.push_back(pose);
        features.push_back(feature);
        global_pos = global_pos * pose;
        // std::cout << global_pos.matrix() << std::endl;
        // for(int i = 0; i < poses.size(); i++) {
        //     std::cout << poses[i].matrix() << std::endl;
        // }
        global_poses.push_back(global_pos);
    }

    std::vector<Sophus::SE3d> Map::getMapPose(){
        return global_poses;
    }
    
    std::vector<std::vector<cv::Point3d>> Map::getMapFeature() {
        return features;
    }

    Sophus::SE3d Map::getGlobalPos() {
        return global_pos;
    }

}