#pragma once
#ifndef MAP_H
#define MAP_H

#include "common.h"

namespace slam {

class Map {

    public:
        Map();
        void insertKeyPoint(const Sophus::SE3d pose, const std::vector<Eigen::Vector3d> feature);
        std::vector<Sophus::SE3d> getMapPose();
        std::vector<std::vector<Eigen::Vector3d>> getMapFeature();

    private:
        std::vector<Sophus::SE3d> poses;
        std::vector<std::vector<Eigen::Vector3d>> features;
};

}

#endif