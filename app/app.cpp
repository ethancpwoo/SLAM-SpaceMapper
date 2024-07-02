#include "backend.h"
#include "frontend.h"
#include "map.h"

int main(int argc, char **argv) {
    cv::Mat k = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    slam::Frontend front_end;
    slam::Backend back_end;
    slam::Map map;

    cv::Mat img_1 = cv::imread(argv[1], cv::IMREAD_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], cv::IMREAD_COLOR);

    std::deque<Sophus::SE3d> active_poses;
    std::deque<std::vector<cv::Point3d>> active_positions;
    std::deque<std::vector<cv::Point2f>> active_pixel_positions;

    front_end.setCamera(k);
    front_end.setMap(map);
    front_end.setImages(img_1, img_2);
    front_end.runFrontEnd();
    front_end.getCurrentBatch(active_poses, active_positions, active_pixel_positions);

    back_end.setCamera(k);

    return 0;
}