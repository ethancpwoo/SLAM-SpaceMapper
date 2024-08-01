#include "loopclose.h"

namespace slam {

LoopClose::LoopClose() {
    vocab.readFromFile("../../test_data/orb_mur.fbow");
}

int LoopClose::findLoop(const cv::Mat &current_descriptor) {
    int index_detected = -1;
    fbow::fBow current_bow = vocab.transform(current_descriptor);
    for(int i = 0; i < prev_bows.size(); i++) {
        if (fbow::fBow::score(current_bow, prev_bows[i]) > 0.8) {
            index_detected = i;
            break;
        }
    }
    return index_detected;
}

void LoopClose::optimize() {


}

}