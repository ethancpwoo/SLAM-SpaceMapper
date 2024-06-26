#pragma once
#ifndef SLAM_FRONTEND_H
#define SLAM_FRONTEND_H

#include "common.h"
#include "map.h"

namespace slam {

class Frontend {
    public:
        Frontend();
        bool setImages(const cv::Mat &img_1, const cv::Mat &img_2);
        bool setMap(const Map &info_map);
        bool setCamera();
        bool runFrontEnd();

    private:
        bool ORBGetFeatures();
        bool getPoseEstimation();
        bool triangulate();
        cv::Point2f pixel2cam(const cv::Point2d &p, const cv::Mat &K);

        Map map;
        int focal_length;
        cv::Point2d principal_point;
        cv::Mat K; 

        cv::Mat img1, img2;
        cv::Mat desc1, desc2;
        
        cv::Mat R, t, H, E, F;
        Eigen::Matrix3d R_eigen;
        Eigen::Vector3d t_eigen;
        Sophus::SE3d pose;

        cv::Mat pts_4d;
        std::vector<cv::KeyPoint> keypnt1, keypnt2;
        std::vector<cv::DMatch> matches, good_matches;
        std::vector<cv::Point2f> points1, points2;
        std::vector<cv::Point2f> pts_1, pts_2;
        std::vector<cv::Point3d> points3d;
        cv::Ptr<cv::FeatureDetector> detector;
        cv::Ptr<cv::DescriptorExtractor> descriptor;
        cv::Ptr<cv::DescriptorMatcher> matcher;
        
};

}

#endif