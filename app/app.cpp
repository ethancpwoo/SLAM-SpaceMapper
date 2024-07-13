#include "backend.h"
#include "frontend.h"
#include "map.h"
#include "viewer.h"
#include <string>

int main(int argc, char **argv) {

    std::vector<std::string> images;
    for(int i = 0; i < 20; i++) {
        std::string fileName;
        fileName = std::to_string((i * 5) + 105);
        std::string zeros((size_t)(5 - fileName.size()), '0'); 
        fileName = "../../dataset/daylight/L_" + zeros + fileName + ".png";
        images.push_back(fileName);
    }

    // --------------------------------------------------------------------------
    //cv::Mat k = (cv::Mat_<double>(3, 3) << 517.3, 0, 318.6, 0, 516.5, 255.3, 0, 0, 1);
    cv::Mat k = (cv::Mat_<double>(3, 3) << 615, 0, 320, 0, 615, 240, 0, 0, 1);
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

    img_1 = cv::imread("../../dataset/daylight/L_00100.png", cv::IMREAD_COLOR);
    img_2 = cv::imread("../../dataset/daylight/L_00105.png", cv::IMREAD_COLOR);
    front_end.setImages(img_1, img_2);
    front_end.runFrontEnd();
    front_end.getCurrentBatch(active_poses, active_positions, active_pixel_positions);

    for(int i = 0; i < 19; i++) {
        
        // img_1 = cv::imread("../../dataset/daylight/L_00001.png", cv::IMREAD_COLOR);
        // img_2 = cv::imread("../../dataset/daylight/L_00015.png", cv::IMREAD_COLOR);

        img_1 = cv::imread(images[i], cv::IMREAD_COLOR);
        img_2 = cv::imread(images[i + 1], cv::IMREAD_COLOR);
        // cv::filter2D(img_1, img_1, CV_8U, kernel);
        // cv::filter2D(img_2, img_2, CV_8U, kernel);
        front_end.setImages(img_1, img_2);
        front_end.runFrontEnd();
        front_end.getCurrentBatch(active_poses, active_positions, active_pixel_positions);
        // std::cout << "front end" << std::endl << active_poses[0].matrix() << std::endl;

        back_end.BundleAdjustment(active_poses, active_positions, active_pixel_positions);
        // std::cout << "back end" << std::endl << active_poses[0].matrix() << std::endl;
        // std::cout << images[i] << std::endl;
        map.insertKeyPoint(active_poses[0], active_positions[0]);
        std::cout << map.getGlobalPos().matrix() << std::endl;

    }

    slam::Viewer viewer;
    viewer.setMap(map);
    viewer.Visualize();
    
    return 0;
}