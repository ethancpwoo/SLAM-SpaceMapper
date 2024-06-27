#pragma once
#ifndef MAP_H
#define MAP_H

#include "common.h"

namespace slam {

class Map {

    public:
        Map();
        void insertKeyPoint(const Sophus::SE3d &pose, const std::vector<Eigen::Vector3d>> &feature);

    private:
        std::vector<Sophus::SE3d> poses;
        std::vector<Sophus::SE3d> active_poses;
        std::vector<std::vector<Eigen::Vector3d>> features;
        std::vector<std::vector<Eigen::Vector3d>> active_features;
};

}

#endif