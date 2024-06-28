#include "map.h"

namespace slam {

    Map::Map() {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        Sophus::SE3d p(mat);
        // info.push_back(p);
    }

    //void Map::insertKeyPoint(const Sophus::SE3d &pose) {
        // info.push_back(pose);
    //}



}