#include "slam/common.h"
#include "slam/frontend.h"

namespace slam {
    
Frontend::Frontend() {

}

Frontend::ORBInit() {
    detector = cv::ORB::create();
    descriptor = cv::ORB::create();
    matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

bool Frontend::ORBGetFeatures() {
    detector->detect(img1, keypnt1);
    detector->detect(img2, keypnt2);
    descriptor->compute(img1, keypnt1, desc1);
    descriptor->compute(img2, keypnt2, desc2);

    matcher->match(desc1, desc2, matches);
    auto min_max = std::minmax_element(matches.begin(), matches.end());
    double max_dist = min_max.first->distance;
    double min_dist = min_max.second->distance;

    for(int i = 0; i < desc_1.rows; i++) {
        if (matches[i].distance <=  std::max(2 * min_dist, 30.0)) {
            good_matches.push_back(matches[i]);
        }
    }

    return true;
}

bool Frontend::getPoseEstimation() {
    for (int i = 0; i < (int) matches.size(); i++) {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }
    F = cv::findFundamentalMat(points1, points2, cv::FM_8POINT);
    E = cv::findEssentialMat(points1, points2, focal_length, principal_point);
    H = cv::findHomography(points1, points2, cv::RANSAC, 3);
    cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
}

bool Frontend::Triangulate() {
    for(cv::DMatch m : matches) {
        pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt, K));
        pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt, K));
    }

    cv::Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

    for(int i = 0; i < pts_4d.cols; i++) {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0);
        cv::Point3d p(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0)
        );
        points.push_back(p);
    }
}
}