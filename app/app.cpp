#include "backend.h"
#include "frontend.h"
#include "map.h"
#include "viewer.h"
#include <string>

int main(int argc, char **argv) {

    // --------------------------------------------------------------------------
    //cv::Mat k = (cv::Mat_<double>(3, 3) << 517.3, 0, 318.6, 0, 516.5, 255.3, 0, 0, 1);
    cv::Mat k = (cv::Mat_<double>(3, 3) << 2714.9, 0, 1296, 0, 2714.29, 972, 0, 0, 1);
    slam::Frontend front_end;
    slam::Backend back_end;
    slam::Map map;

    cv::Mat img_1;
    cv::Mat img_2;

    std::deque<Sophus::SE3d> active_poses;
    std::deque<std::vector<cv::Point3d>> active_positions;
    std::deque<std::vector<cv::Point2d>> active_pixel_positions;

    front_end.setCamera(k);
    front_end.setMap(map);
    back_end.setCamera(k);

    cv::Mat kernel = (cv::Mat_<double>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);

    img_1 = cv::imread("../../test_data/test4.jpg", cv::IMREAD_COLOR);
    img_2 = cv::imread("../../test_data/test5.jpg", cv::IMREAD_COLOR);
    front_end.setImages(img_1, img_2);
    front_end.runFrontEnd();
    front_end.getCurrentBatch(active_poses, active_positions, active_pixel_positions);

    for(int i = 5; i < 14; i++) {
        
        std::string img_name_1 = "../../test_data/test";
        img_name_1.append(std::to_string(i));
        img_name_1.append(".jpg");

        std::string img_name_2 = "../../test_data/test";
        img_name_2.append(std::to_string(i + 1));
        img_name_2.append(".jpg");

        img_1 = cv::imread(img_name_1, cv::IMREAD_COLOR);
        img_2 = cv::imread(img_name_2, cv::IMREAD_COLOR);
        // cv::filter2D(img_1, img_1, CV_8U, kernel);
        // cv::filter2D(img_2, img_2, CV_8U, kernel);

        front_end.setImages(img_1, img_2);
        front_end.runFrontEnd();
        front_end.getCurrentBatch(active_poses, active_positions, active_pixel_positions);
        // std::cout << "front end" << std::endl << active_poses[0].matrix() << std::endl;

        back_end.BundleAdjustment(active_poses, active_positions, active_pixel_positions);
        // std::cout << "back end" << std::endl << active_poses[0].matrix() << std::endl;
        // std::cout << images[i] << std::endl;
        if (i > 10) {
            map.insertKeyPoint(active_poses.front(), active_positions.front());
        }

    }

    for(int i = 0; i < 5; i++) {
        map.insertKeyPoint(active_poses.front(), active_positions.front());
        active_poses.pop_front();
        active_positions.pop_front(); 
    }

    slam::Viewer viewer;
    viewer.setMap(map);
    viewer.Visualize();
    
    return 0;
}