#include "frontend.h"

namespace slam {
    
Frontend::Frontend() {
    detector = cv::ORB::create();
    descriptor = cv::ORB::create();
    matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    // Temporary test values
    focal_length = 521;
    principal_point = cv::Point2d(325.1, 249.7);
    // Intrisic matrix 
    K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
}

bool Frontend::setMap(const Map &info_map) {
    map = info_map;
    return true;
}

bool Frontend::setImages(const cv::Mat &img_1, const cv::Mat &img_2) {
    img1 = img_1; 
    img2 = img_2;
    assert(img_1.data != nullptr && img_2.data != nullptr);
    return true;
}

bool Frontend::setCamera(const cv::Mat &k) {
    K = k;
    return true;
}

bool Frontend::getCurrentBatch(
    std::deque<Sophus::SE3d> &active_poses,
    std::deque<std::vector<cv::Point3d>> &active_positions,
    std::deque<std::vector<cv::Point2d>> &active_pixel_positions) {

    if(active_poses.size() >= 5) {
        active_poses.pop_front();
        active_positions.pop_front();
        active_pixel_positions.pop_front();
    }

    active_poses.push_back(pose);
    active_positions.push_back(points3d);
    active_pixel_positions.push_back(points2);

    return true;
}

bool Frontend::runFrontEnd() {
    ORBGetFeatures();
    getPoseEstimation(); 
    triangulate();
    return true;
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

    for(int i = 0; i < desc1.rows; i++) {
        if (matches[i].distance <=  std::max(2 * min_dist, 30.0)) {
            good_matches.push_back(matches[i]);
        }
    }
    return true;
}

bool Frontend::getPoseEstimation() {
    for (int i = 0; i < (int) matches.size(); i++) {
        points1.push_back(keypnt1[matches[i].queryIdx].pt);
        points2.push_back(keypnt2[matches[i].trainIdx].pt);
    }
    F = cv::findFundamentalMat(points1, points2, cv::FM_8POINT);
    E = cv::findEssentialMat(points1, points2, focal_length, principal_point);
    H = cv::findHomography(points1, points2, cv::RANSAC, 3);
    cv::recoverPose(E, points1, points2, R, t, focal_length, principal_point);
    cv::cv2eigen(R, R_eigen);
    cv::cv2eigen(t, t_eigen);
    std::cout << R_eigen.matrix() << std::endl;
    std::cout << t_eigen.matrix() << std::endl;
    pose = Sophus::SE3d(R_eigen, t_eigen);
    std::cout << pose.log().transpose() << std::endl;
    return true;
}

bool Frontend::triangulate() {
    //find out what pixel2cam does, 
    for(cv::DMatch m : matches) {
        pts_1.push_back(pixel2cam(keypnt1[m.queryIdx].pt, K));
        pts_2.push_back(pixel2cam(keypnt2[m.trainIdx].pt, K));
    }
    cv::Mat T1 = (cv::Mat_<float>(3, 4) << 
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0);
    cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), R.at<double>(0, 0),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), R.at<double>(1, 0),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), R.at<double>(2, 0)
    );

    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

    for(int i = 0; i < pts_4d.cols; i++) {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0);
        cv::Point3d p(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0)
        );
        //std::cout << "x: " << p.x << " y: " << p.y << " z: " << p.z << std::endl;
        points3d.push_back(p);
    }
    return true;
}

cv::Point2d Frontend::pixel2cam(const cv::Point2d &p, const cv::Mat &K) {
    return cv::Point2d 
    (
        (p.x - K.at<double>(0, 2) / K.at<double>(0, 0)),
        (p.y - K.at<double>(1, 2) / K.at<double>(1, 1))
    );
}

}