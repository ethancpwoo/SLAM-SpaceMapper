#include <iostream>
#include <opencv2/opencv.hpp>

void find_feature_matches(
    const cv::Mat &img_1, const cv::Mat &img_2, 
    std::vector<cv::KeyPoint> &keypoints_1, 
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches
);

void pose_estimation(
    const std::vector<cv::KeyPoint> &keypoints_1,
    const std::vector<cv::KeyPoint> &keypoints_2,
    const std::vector<cv::DMatch> &matches,
    cv::Mat &R, cv::Mat &t
);

void triangulation(
    const std::vector<cv::KeyPoint> &keypoint_1,
    const std::vector<cv::KeyPoint> &keypoint_2,
    const std::vector<cv::DMatch> &matches,
    const cv::Mat &R, const cv::Mat &t,
    std::vector<cv::Point3d> &points
);

inline cv::Scalar get_color(float depth) {
    float up_th = 50, low_th = 10, th_range = up_th - low_th;
    if (depth > up_th) depth = up_th;
    if (depth < low_th) depth = low_th;
    return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
}

cv::Point2f pixel2cam(const cv::Point2d &p, const cv::Mat &k);

int main(int argc, char ** argv) {
    cv::Mat img_1 = cv::imread(argv[1], cv::LOAD_IMAGE_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], cv::LOAD_IMAGE_COLOR);

    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    std::vector<cv::DMatch> matches;

    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    
    cv::Mat R, t;
    pose_estimation(keypoints_1, keypoints_2, matches, R, t);

    std::vector<cv::Point3d> points;
    triangulation(keypoints_1, keypoints_2, matches, R, t, points);

    cv::Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    cv::Mat img1_plot = img1_clone();
    cv::Mat img2_plot = img2_clone();
    for(int i = 0; i < matches.size(); i++) {
        float depth1 = points[i].z;
        std::cout << "depth: " << depth1 << std::endl;
        cv::Point2d pt1_cam = pixel2cam(keypoints_1[matches[i].trainIdx].pt, 2, get_color(depth2), 2);
        cv::circle(img1_plot, keypoints_1[matches[i].queryIdx].pt, 2, get_color(depth1), 2);

        cv::Mat pt2_trans = R * (Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
        float depth2 = pt2_trans.at<double>(2, 0);
        cv::circle(img2_plot, keypoints_2[matches[i].trainIdx].pt, 2, get_color(depth2), 2);
    }
    cv::imshow("img 1", img1_plot);
    cv::imshow("img 2", img2_plot);

    cv::waitKey();
    return 0;

void find_feature_matches(const cv::Mat &img_1, const cv::Mat &img_2,
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &Keypoints_2,
    std::vector<cv::DMatch> &matches) {
        cv::Mat descriptors_1, descriptors_2;
        cv::Ptr<FeatureDetector> detector = ORB::create(); 
        cv::Ptr<DescriptorExtractor> descriptor = ORB::create();

        cv::Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

        detector->detect(img_1, keypoints_1);
        detector->detect(img_2, keypoints_2);
        
        descriptor->compute(img_1, keypoints_1);
        descriptor->compute(img_2, keypoints_2);
        
        std::vector<DMatch> match;

        matcher->match(descriptor_1, descriptor_2, match);

        double min_dist = 10000, max_dist = 0;
        for (int i = 0; i < descriptors_1.rows; i++) {
            double dist = match[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }
        
        std::cout << "Max Dist: " << max_dist << std::endl;
        std::cout << "Min Dist: " << min_dist << std::endl;

        for(int i = 0; i < descriptors_1.rows; i++) {
            if (match[i].distance <= max(2 * min_dist, 30.0)) {
                matches.push_back(match[i]);
            }
        }
    }

void pose_estimation(
    const std::vector<KeyPoint> &keypoints_1, 
    const std::vector<KeyPoint> &keypoints_2, 
    std::vector<DMatch> matches, 
    cv::Mat &R, cv::Mat &t) {
    
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

void triangulation(
    const std::vector<cv::KeyPoint> &keypoint_1,
    const std::vector<cv::KeyPoint> &keypoint_2,
    const std::vector<cv::DMatch> &matches,
    const cv::Mat &R, const cv::Mat &t,
    std::vector<Point3d> &points) {
    cv::Mat T1 = (Mat_<float>(3, 4) << 
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0);
    cv::Mat T2 = (Mat_<float>(3, 4) <<
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), R.at<double>(0, 0),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), R.at<double>(1, 0),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), R.at<double>(2, 0),
    );

    cv::Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    std::vector<cv::Point2f> pts_1, pts_2;
    for(cv::DMatch m : matches) {
        pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt, K));
        pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].ptm K));
    }

    cv::Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

    for(int i = 0; i < pts_4d.cols; i++) {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0);
        Point3d p(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0)
        );
        points.push_back(p);
    }
}

Point2f pixel2cam(const cv::Point2d &p, const cv::Mat &K) {
    return Point2f 
    (
        (p.x - K.at<double>(0, 2) / K.at<double>(0, 0)),
        (p.y - K.at<double>(1, 2) / K.at<double>(1, 1))
    );
}