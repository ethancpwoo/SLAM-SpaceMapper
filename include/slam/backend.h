#pragma once
#ifndef SLAM_BACKEND_H
#define SLAM_BACKEND_H

#include "slam/common.h"

namespace slam {

class Backend {
    public:
        Backend();        

    private:
        bool BundleAdjustment();

        const int point_block_size;
        const int camera_block_size;
        double *camera;
        double *point;
        double *points;
        double *cameras;
        ceres::CostFunction *cost_function;
        ceres::LossFunction *loss_function;
        
        
};

}

#endif
