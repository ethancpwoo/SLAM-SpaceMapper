#include "viewer.h"

namespace slam { //draws upside down or something mathematically wrong...

void Viewer::Close() {
    viewer_running_ = false;
    // viewer_thread_.join();
}

void Viewer::setMap(Map &map){
    map_ = map;
}

// void Viewer::UpdateMap() {
//     std::unique_lock<std::mutex> lck(viewer_data_mutex_);
//     assert(map_ != nullptr);
//     active_keyframes_ = map_->GetActiveKeyFrames();
//     active_landmarks_ = map_->GetActiveMapPoints();
//     map_updated_ = true;
// }

void Viewer::Visualize() {
    pangolin::CreateWindowAndBind("MySLAM", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState vis_camera(
        pangolin::ProjectionMatrix(1024, 768, 615, 615, 320, 240, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& vis_display =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(vis_camera));

    const float blue[3] = {0, 0, 1};
    float green[3] = {0, 1, 0};

    for(int i = 0; i < 9; i++) {
        float* color = new float[3]{0.1f * i, 0, (1.0f/i)}; 
        colors.push_back(color);
    }

    current_pose = map_.getGlobalPos();

    while (!pangolin::ShouldQuit() && viewer_running_) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        vis_display.Activate(vis_camera);

        std::unique_lock<std::mutex> lock(viewer_data_mutex_);
        // DrawFrame(current_pose, green);
        FollowCurrentFrame(vis_camera);

        // cv::Mat img = PlotFrameImage();
        // cv::imshow("image", img);
        // cv::waitKey(1);

        DrawMapPoints();

        pangolin::FinishFrame();
        // usleep(5000);
    }

    for (auto& color : colors) {
        delete[] color;
    }
}

// cv::Mat Viewer::PlotFrameImage() {
//     cv::Mat img_out;
//     cv::cvtColor(current_frame_->left_img_, img_out, cv::CV_GRAY2BGR);
//     for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
//         if (current_frame_->features_left_[i]->map_point_.lock()) {
//             auto feat = current_frame_->features_left_[i];
//             cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 250, 0),
//                        2);
//         }
//     }
//     return img_out;
// }

void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera) {
    Sophus::SE3d Twc = current_pose.inverse();
    pangolin::OpenGlMatrix m;
    auto Twc_mat = Twc.matrix();
    m.m[0] = Twc_mat(0, 0);
    m.m[1] = Twc_mat(1, 0);
    m.m[2] = Twc_mat(2, 0);
    m.m[3] = Twc_mat(3, 0);

    m.m[4] = Twc_mat(0, 1);
    m.m[5] = Twc_mat(1, 1);
    m.m[6] = Twc_mat(2, 1);
    m.m[7] = Twc_mat(3, 1);

    m.m[8] = Twc_mat(0, 2);
    m.m[9] = Twc_mat(1, 2);
    m.m[10] = Twc_mat(2, 2);
    m.m[11] = Twc_mat(3, 2);

    m.m[12] = Twc_mat(0, 3);
    m.m[13] = Twc_mat(1, 3);
    m.m[14] = Twc_mat(2, 3);
    m.m[15] = Twc_mat(3, 3);

    vis_camera.Follow(m, true);
}

void Viewer::DrawFrame(Sophus::SE3d &pose, const float* color) {
    Sophus::SE3d Twc = pose.inverse();
    const float sz = 1.0;
    const int line_width = 2.0;
    const float fx = 2714.9;
    const float fy = 2714.9;
    const float cx = 1296;
    const float cy = 972;
    const float width = 1080;
    const float height = 768;

    glPushMatrix();

    Sophus::Matrix4f m = Twc.matrix().template cast<float>();
    glMultMatrixf((GLfloat*)m.data());

    // if (color == nullptr) {
    //     glColor3f(1, 0, 0);
    // } else
    //     glColor3f(color[0], color[1], color[2]);

    glColor3f(color[0], color[1], color[2]);

    glLineWidth(line_width);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glEnd();
    glPopMatrix();
}

void Viewer::DrawMapPoints() {
    const float red[3] = {1.0, 0, 0};
    int index = 0;
    
    int color_index = 0;
    for (auto& pos : map_.getMapPose()) {
        DrawFrame(pos, colors[color_index]);
        color_index += 1;
    }

    // glPointSize(2);
    // glBegin(GL_POINTS);
    // for (auto& landmarks : map_.getMapFeature()) {
    //     auto pos = landmarks;
    //     for(auto& point : pos) {
    //         std::cout << point << std::endl; 
    //         glColor3f(red[0], red[1], red[2]);
    //         glVertex3d(point.x, point.y, point.z);
    //     }
    // }
    // glEnd();
}

}  // namespace myslam