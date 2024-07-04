#include "loopclose.h"

namespace slam {

LoopClose::LoopClose() {
    vocab = new fbow::Vocabulary();
    // read in images + descriptors
    // std::vector<cv::Mat> images;
}

bool LoopClose::insertDict(const cv::Mat descriptor) {

    fbow::fBow v1;
    v1 = vocab->transform(descriptor); 
    return true;
    //double score = fbow::fBow::score(v1, v2);
    // To be honest, not sure if loop closure is needed for now, this seems like once Frontend and Backend work then global positioning is required.

}

}