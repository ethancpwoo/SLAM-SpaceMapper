#include "frontend.h"

int main(int argc, char **argv) {
    slam::Frontend front_end;
    cv::Mat img_1 = cv::imread(argv[1], cv::IMREAD_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], cv::IMREAD_COLOR);
    front_end.setImages(img_1, img_2);
    front_end.runFrontEnd();
    return 0;
}