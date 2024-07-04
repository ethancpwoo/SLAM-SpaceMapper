#ifndef LOOPCLOSE_H
#define LOOPCLOSE_H

#include "common.h"
#include "fbow/fbow.h"

namespace slam {

class LoopClose {

    public:
        LoopClose();
        bool insertDict(const cv::Mat descriptor);

    private:
        fbow::Vocabulary* vocab;

};

}

#endif