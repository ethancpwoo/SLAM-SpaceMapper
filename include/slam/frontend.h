#pragma once
#ifndef SLAM_FRONTEND_H
#define SLAM_FRONTEND_H

#include "slam/common.h"

namespace slam {

class Frontend {
    public:
        Frontend();
        bool runFrontEnd();
        

    private:
        bool ORBInit();
        bool ORBGetFeatures();
        bool getPoseEstimation();
        bool Triangulate();

        cv::Mat img1, img2;
        cv::Mat desc1, desc2;
        cv::Mat R, t, E, F;
        std::vector<cv::KeyPoint> keypnt1, keypnt2;
        std::vector<cv::DMatch> matches, good_matches;
        std::vector<cv::Point2f> points1, points2;
        cv::Ptr<cv::FeatureDetector> detector;
        cv::Ptr<cv::DescriptorExtractor> descriptor;
        cv::Ptr<cv::DescriptorMatcher> matcher;
    
};

}

#endif  // MYSLAM_FRONTEND_H