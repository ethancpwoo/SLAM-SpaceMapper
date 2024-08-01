#ifndef LOOPCLOSE_H
#define LOOPCLOSE_H

#include <fbow/fbow.h>

#include "common.h"

namespace slam {

class LoopClose {

    public:
        LoopClose();
        int findLoop(const cv::Mat &current_descriptor);
        void optimize();

    private:
        fbow::Vocabulary vocab;
        std::vector<fbow::fBow> prev_bows;

};

}

#endif