#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>
#include <unistd.h>

#include "common.h"
#include "map.h"

namespace slam {

class Viewer {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    // Viewer();
    void setMap(Map &map);
    void Visualize();
    void Close();
    // void AddCurrentFrame(Frame::Ptr current_frame);
    // void UpdateMap();

   private:
    
    void DrawFrame(Sophus::SE3d &pose, const float* color);
    void DrawMapPoints();
    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);
    // cv::Mat PlotFrameImage();
    Sophus::SE3d current_pose;
    // Frame::Ptr current_frame_ = nullptr;
    Map map_;
    std::thread viewer_thread_;
    bool viewer_running_ = true;

    std::vector<Sophus::SE3d> active_positions;
    std::vector<std::vector<cv::Point3d>> active_landmarks;
    bool map_updated_ = false;

    std::mutex viewer_data_mutex_;
};
}

#endif  // MYSLAM_VIEWER_H