#include "backend.h"
#include "frontend.h"
#include "map.h"
#include <string>

int main(int argc, char **argv) {

    // std::vector<std::string> images;
    // for(int i = 0; i < 9; i++) {
    //     std::string fileName;
    //     fileName = std::to_string(i + 1);
    //     fileName = "../../dataset/daylight/L_0000" + fileName + ".png";
    //     images.push_back(fileName);
    // }

    // --------------------------------------------------------------------------
    //cv::Mat k = (cv::Mat_<double>(3, 3) << 517.3, 0, 318.6, 0, 516.5, 255.3, 0, 0, 1);
    cv::Mat k = (cv::Mat_<double>(3, 3) << 615, 0, 320, 0, 615, 240, 0, 0, 1);
    slam::Frontend front_end;
    slam::Backend back_end;
    slam::Map map;

    // cv::Mat img_1 = cv::imread(argv[1], cv::IMREAD_COLOR);
    // cv::Mat img_2 = cv::imread(argv[2], cv::IMREAD_COLOR);

    cv::Mat img_1;
    cv::Mat img_2;

    std::deque<Sophus::SE3d> active_poses;
    std::deque<std::vector<cv::Point3d>> active_positions;
    std::deque<std::vector<cv::Point2d>> active_pixel_positions;

    front_end.setCamera(k);
    front_end.setMap(map);
    back_end.setCamera(k);

    cv::Mat kernel = (cv::Mat_<double>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);

    // for(int i = 0; i < 9; i++) {

        img_1 = cv::imread("../../dataset/daylight/L_00001.png", cv::IMREAD_COLOR);
        img_2 = cv::imread("../../dataset/daylight/L_00015.png", cv::IMREAD_COLOR);
        // img_1 = cv::imread(images[i], cv::IMREAD_COLOR);
        // img_2 = cv::imread(images[i + 1], cv::IMREAD_COLOR);
        // cv::filter2D(img_1, img_1, CV_8U, kernel);
        // cv::filter2D(img_2, img_2, CV_8U, kernel);
        front_end.setImages(img_1, img_2);
        front_end.runFrontEnd();
        front_end.getCurrentBatch(active_poses, active_positions, active_pixel_positions);
        // std::cout << active_poses[0].matrix() << std::endl;

        back_end.BundleAdjustment(active_poses, active_positions, active_pixel_positions);
        // std::cout << active_poses[0].matrix() << std::endl;
        // std::cout << i << std::endl;

    // }

    // back_end.setCamera(k);
    // back_end.BundleAdjustment(active_poses, active_positions, active_pixel_positions);

    // std::cout << active_poses[0].matrix() << std::endl;
    
    return 0;
}