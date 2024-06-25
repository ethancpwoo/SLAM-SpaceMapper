#include "DBoW3/DBoW.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <vector>
#include <string>

int main(int argc, char **argv) {
    std::vector<cv::Mat> images;
    for(int i = 0; i < num_images; i++) {
    }

    cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
    std::vector<cv::Mat> descriptors;
    for(cv::Mat &image : images) {
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptor;
        detector->detectAndCompute(image, cv::Mat(), keypoints, descriptor);
        descriptors.push_back(descriptor)
    }

    DBoW3::Vocabulary vocab;
    vocab.create(descriptors);
    vocab.save("vocabulary.yml.gz");
    return 0;
}
