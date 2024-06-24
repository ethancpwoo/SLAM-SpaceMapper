#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

void pose_estimation(std::vector<KeyPoint> keypoints_1, std::vector<KeyPoint> keypoints_2, std::vector<DMatch> matches, cv::Mat &R, cv::Mat &t);

cv::Point2d pixel2cam(const Point2d &p, const cv::Mat &K);

int main(int argc, char **argv) {
    cv::Mat img_1 = cv::imread(argv[1], cv::LOAD_IMAGE_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], cv::LOAD_IMAGE_COLOR);
    assert(img_1.data && img_2.data && "Cannot load image");

    std::vector<cv::KeyPoint> keypoints_1, keypoint_2;
    std::vector<cv::Dmatch> matches;
    
    // find features

    cv::Mat R, t;
    pose_estimation(keypoints_1, keypoints_2, matches, R, t);

    cv::Mat t_x = (
        Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
        t.at<double>(2, 0), 0, -t.at<double>(0, 0),
        -t.at<double>(1, 0), t.at<double>(0, 0), 0
    );

    cv::Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    for(cv::DMatch m: matches) {
        cv::Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        cv::Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
        cv::Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
        cv::Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
        cv::Mat d = y2.t() * t_x * R * y1;

        std::cout << "epipolar constraint = " << d << std::endl;
    }
    return 0;
}

cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K) {
    return cv::Point2d (
        (p.x - K.at<double>(0, 2)) / (K.at<double>(0, 0)),
        (p.y - L.at<double>(1, 2)) / (K.at<double>(1, 1))
    );
}

void pose_estimation(std::vector<KeyPoint> keypoints_1, std::vector<KeyPoint> keypoints_2, std::vector<DMatch> matches, cv::Mat &R, cv::Mat &t) {
    cv::Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;

    for (int i = 0; i < (int) matches.size(); i++) {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    cv::Mat fundemental_matrix;
    fundemental_matrix = cv::findFundamentalMat(points1, points2, cv::CV_FM_8POINT);
    std::cout << "fundamental_matrix is " << std::endl << fundamental_matrix << std::endl;

    cv::Point2d principal_point(325.1, 249.7);
    double focal_length = 521;
    cv::Mat essential_matrix;
    essential_matrix = cv::findEssentialMat(points1, points2, focal_length, principal_point);
    std::cout << "essential_matrix is " << std::endl << essential_matrix << std::endl;

    cv::Mat homography_matrix;
    homography_matrix = findHomography(points1, points2, cv::RANSAC, 3);
    std::cout << "homography_matrix is " << std::endl << homography_matrix << std::endl;

    cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    std::cout << "R is " << std::endl << R << std::endl;
    std::cout << "t is " << std::endl << t << std::endl;
    
}
