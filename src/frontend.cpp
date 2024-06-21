#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>

int main(int argc, char **argv) {

    cv::Mat img_1 = cv::imread(argv[1], cv::IMREAD_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], cv::IMREAD_COLOR);

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
    cv::imshow("ORB features", out_img);

    std::vector<cv::DMatch> matches;
    matcher->match(desc_1, desc_2, matches);

    auto min_max = std::minmax_element(matches.begin(), matches.end());
    double max_dist = min_max.first->distance;
    double min_dist = min_max.second->distance;

    std::cout << "Max dist: " << max_dist << std::endl;
    std::cout << "Min dist: " << min_dist << std::endl;

    std::vector<cv::DMatch> good_matches;
    for(int i = 0; i < desc_1.rows; i++) {
        if (matches[i].distance <=  std::max(2 * min_dist, 30.0)) {
            good_matches.push_back(matches[i]);
        }
    }

    cv::Mat img_match;
    cv::Mat img_goodmatches;

    cv::drawMatches(img_1, keypnt_1, img_2, keypnt_2, matches, img_match);
    cv::drawMatches(img_1, keypnt_1, img_2, keypnt_2, good_matches, img_goodmatches);

    cv::imshow("Matches", img_match);
    cv::imshow("Good matches", img_goodmatches);

    cv::waitKey(0);

    return 0;
}