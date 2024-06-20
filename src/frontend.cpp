#include <iostream>
#include <opencv2/core/core.hpp>
#include <chrono>

int main(int argc, char **argv) {

    cv::Mat img_1 = cv::imread(argv[1], cv::CV_LOAD_IMAGE_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], cv::CV_LOAD_IMAGE_COLOR);

    assert(img_1.data != nullptr && img_2.data != nullptr);
    
    std::vector<cv::KeyPoint> keypnt_1, keypnt_2;
    cv::Mat desc_1, desc_2;
    
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    // Oriented FAST
    detector->detect(img_1, keypnt_1);
    detector->detect(img_2, keypnt_2);

    // BRIEF descriptor
    descriptor->compute(img_1, keypnt_1, desc_1);
    descriptor->compute(img_2, keypnt_2, desc_2);

    cv::Mat out_img;
    cv::drawKeypoints(img_1, keypnt_1, out_img, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    

}