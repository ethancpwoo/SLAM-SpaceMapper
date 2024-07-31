#include "frontend.h"

namespace slam {
    
Frontend::Frontend() {
    detector = cv::ORB::create();
    descriptor = cv::ORB::create();
    matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    K = cv::Mat_<double>(3, 3);
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
    focal_length = 2714.29;
    principal_point = cv::Point2d(1296, 972);
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
    
    keypnt1.clear();
    keypnt2.clear();
    matches.clear();
    good_matches.clear();
    points1.clear();
    points2.clear();
    pts_1.clear();
    pts_2.clear();
    points3d.clear();

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
    auto min_max = std::minmax_element(matches.begin(), matches.end(), 
        [](const cv::DMatch &m1, const cv::DMatch &m2) {return m1.distance < m2.distance; });
    double max_dist = min_max.first->distance;
    double min_dist = min_max.second->distance;
    // std::cout << max_dist << std::endl; 
    // std::cout << min_dist << std::endl;
    for(int i = 0; i < desc1.rows; i++) {
        // std::cout << matches[i].distance << std::endl;
        if (matches[i].distance <=  20 ) { //std::max(2 * min_dist, 30.0)
            good_matches.push_back(matches[i]);
        }
    }
    cv::Mat img_goodmatches;

    // cv::drawMatches(img1, keypnt1, img2, keypnt2, good_matches, img_goodmatches);

    // cv::namedWindow("Good matches", cv::WINDOW_NORMAL);
    // cv::imshow("Good matches", img_goodmatches);
    // cv::waitKey(0);

    return true;
}

bool Frontend::getPoseEstimation() {
    // std::cout << good_matches.size() << std::endl;
    for (int i = 0; i < good_matches.size(); i++) {
        points1.push_back(keypnt1[good_matches[i].queryIdx].pt);
        points2.push_back(keypnt2[good_matches[i].trainIdx].pt);
    }
    // std::cout << points1 << std::endl;
    // std::cout << points2 << std::endl;

    F = cv::findFundamentalMat(points1, points2, cv::RANSAC);
    E = cv::findEssentialMat(points1, points2, focal_length, principal_point);
    H = cv::findHomography(points1, points2, cv::RANSAC, 3);
    cv::recoverPose(E, points1, points2, R, t, focal_length, principal_point);
    //t = 5 * t;
    cv::cv2eigen(R, R_eigen);
    cv::cv2eigen(t, t_eigen);
    // std::cout << R_eigen.matrix() << std::endl;
    // std::cout << t_eigen.matrix() << std::endl;
    pose = Sophus::SE3d(R_eigen, t_eigen);
    // std::cout << pose.log().transpose() << std::endl;
    // std::cout << pose.matrix() << std::endl;
    return true;
}

bool Frontend::triangulate() {
    //find out what pixel2cam does, 
    for(cv::DMatch m : good_matches) {
        pts_1.push_back(pixel2cam(keypnt1[m.queryIdx].pt, K));
        pts_2.push_back(pixel2cam(keypnt2[m.trainIdx].pt, K));
    }
    cv::Mat T1 = (cv::Mat_<double>(3, 4) << 
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0);
    cv::Mat T2 = (cv::Mat_<double>(3, 4) <<
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
    );

    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

    for(int i = 0; i < pts_4d.cols; i++) {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<double>(3, 0);
        cv::Point3d p(
            x.at<double>(0, 0),
            x.at<double>(1, 0),
            x.at<double>(2, 0)
        );
        // std::cout << "x: " << p.x << " y: " << p.y << " z: " << p.z << std::endl;
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