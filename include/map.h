#pragma once
#ifndef MAP_H
#define MAP_H

#include "common.h"

namespace slam {

class Map {

    public:
        Map();
        void insertKeyPoint(Sophus::SE3d &pose, std::vector<cv::Point3d> &feature);
        std::vector<Sophus::SE3d> getMapPose();
        std::vector<std::vector<cv::Point3d>> getMapFeature();
        Sophus::SE3d getGlobalPos();

    private:
        std::vector<Sophus::SE3d> poses;
        std::vector<Sophus::SE3d> global_poses;
        std::vector<std::vector<cv::Point3d>> features;
        Sophus::SE3d global_pos;
};

}

#endif