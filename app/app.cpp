#include "backend.h"
#include "frontend.h"
#include "loopclose.h"
#include "map.h"
#include "viewer.h"
#include <string>

int main(int argc, char **argv) {

    bool init_done = false;

    cv::Mat k = (cv::Mat_<double>(3, 3) << 2714.9, 0, 1296, 0, 2714.29, 972, 0, 0, 1);
    slam::Frontend front_end;
    slam::Backend back_end;
    slam::LoopClose loop_closure;
    slam::Map map;

    cv::Mat img_1;
    cv::Mat img_2;
    cv::Mat descriptor1, descriptor2;

    std::deque<Sophus::SE3d> active_poses;
    std::deque<std::vector<cv::Point3d>> active_positions;
    std::deque<std::vector<cv::Point2d>> active_pixel_positions;

    front_end.setCamera(k);
    front_end.setMap(map);
    back_end.setCamera(k);
    loop_closure.setCamera(k);

    cv::Mat kernel = (cv::Mat_<double>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);

    for(int i = 4; i < 13; i++) {
        
        std::string img_name_1 = "../../test_data/test";
        img_name_1.append(std::to_string(i));
        img_name_1.append(".jpg");

        std::string img_name_2 = "../../test_data/test";
        img_name_2.append(std::to_string(i + 1));
        img_name_2.append(".jpg");

        img_1 = cv::imread(img_name_1, cv::IMREAD_COLOR);
        img_2 = cv::imread(img_name_2, cv::IMREAD_COLOR);

        front_end.setImages(img_1, img_2);
        front_end.runFrontEnd();
        front_end.getCurrentDescriptors(descriptor1, descriptor2);
        front_end.getCurrentBatch(active_poses, active_positions, active_pixel_positions);
        // std::cout << "front end" << std::endl << active_poses[0].matrix() << std::endl;

        back_end.BundleAdjustment(active_poses, active_positions, active_pixel_positions);
        // std::cout << "back end" << std::endl << active_poses.back().matrix() << std::endl;
        if (i == 4) loop_closure.findLoop(descriptor1);
        int loop_index = loop_closure.findLoop(descriptor2);
        if (loop_index) loop_closure.optimize(active_poses.front(), map.getRelativePose()[loop_index]);

        if (active_poses.size() >= 5) init_done = true;
        if (init_done) map.insertKeyPoint(active_poses.front(), active_positions.front());

    }
    
    for(int i = 0; i < active_poses.size(); i++) {
        map.insertKeyPoint(active_poses.front(), active_positions.front());
        active_poses.pop_front();
        active_positions.pop_front();
    }

    slam::Viewer viewer;
    viewer.setMap(map);
    viewer.Visualize();
    
    return 0;
}