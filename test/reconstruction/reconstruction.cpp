#include <iostream>
#include <vector>
#include <fstream>

#include <boost/timer.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

const int boarder = 20; 
const int width = 640;
const int height = 480;
const double fx = 481.2f;
const double fy = -480.0f;
const double cx = 319.5f;
const double cy = 239.5f;
const int ncc_window_size = 3;
const int ncc_area = (2 * ncc_window_size + 1) * (2 * ncc_window_size + 1);
const double min_cov = 0.1;
const double max_cov = 10;

bool readDatasetFiles(
    const string &path,
    std::vector<string> &color_image_files,
    std::vector<Sophus::SE3d> &poses,
    cv::Mat &ref_depth
);

bool update(
    const cv::Mat &ref,
    const cv::Mat &curr,
    const Sophus::SE3d &T_C_R,
    cv::Mat &depth,
    cv::Mat &depth_cov2
);

bool epipolarSearch(
    const cv::Mat &ref,
    const cv::Mat &curr,
    const Sophus::SE3d &T_C_R,
    const eigen::Vector2d &pt_ref,
    const double &depth_mu,
    const double &depth_cov,
    eigen::Vector2d &pt_curr,
    eigen::Vector2d &epipolar_direction
);

bool updateDepthFilter(
    const eigen::Vector2d &pt_ref,
    const eigen::Vector2d &pt_curr,
    const Sophus::SE3d &T_C_R,
    const eigen::Vector2d &epipolar_direction,
    cv::Mat &depth,
    cv::Mat &depth_cov2
);

double NCC(
    const cv::Mat &ref,
    const cv::Mat &curr,
    const eigen::Vector2d &pt_ref,
    const eigen::Vector2d &pt_curr
);

void plotDepth(
    const cv::Mat &depth_truth, 
    const cv::Mat &depth_estimate
);

void showEpipolarMatch(
    const cv::Mat &ref,
    const cv::Mat &curr,
    const eigen::Vector2d &px_ref,
    const eigen::Vector2d &px_curr
);

void showEpipolarLine(
    const cv::Mat &ref,
    const cv::Mat &curr,
    const eigen::Vector2d &px_ref,
    const eigen::Vector2d &px_min_curr,
    const eigen::Vector2d &px_max_curr
);

void evaluateDepth(
    const cv::Mat &depth_truth,
    const cv::Mat &depth_estimate
);

inline double getBilinearInterpolatedValue(const cv::Mat &img, const eigen::Vector2d &pt) {
    uchar *d = &img.data[int(pt(1, 0)) * img.step + int(pt(0, 0))];
    double xx = pt(0, 0) - floor(pt(0, 0));
    double yy = pt(1, 0) - floor(pt(1, 0));
    return (
        (1 - xx) * (1 - yy) * double(d[0]) +
        xx * (1 - yy) * double(d[1]) +
        (1 - xx) * yy * double(d[img.step]) +
        xx * yy * double(d[img.step + 1])) / 255.0;
}

inline eigen::Vector3d px2cam(const eigen::Vector2d px) {
    return eigen::Vector3d (
        (px(0, 0) - cx) / fx,
        (px(1, 0) - cy) / fy,
        1
    );
}

inline eigen::Vector2d cam2px(const eigen::Vector3d p_cam) {
    return eigen::Vector2d(
        p_cam(0, 0) * fx / p_cam(2, 0) + cx,
        p_cam(1, 0) * fy / p_cam(2, 0) + cy
    );
}

inline bool inside(const eigen::Vector2d &pt) {
    return 
        pt(0, 0) >= boarder && pt(1, 0) >= boarder &&
        pt(0, 0) + boarder < width && pt(1, 0) + boarder <= height;

}

int main(int argc, char **argv) {
    std::vector<string> color_image_files;
    std::vector<Sophus::SE3d> poses_TWC;
    cv::Mat ref_depth;
    bool ret = readDatasetFiles(argv[1], color_image_files, poses_TWC, ref_depth);
    if (ret == false) {
        return -1; 
    }

    cv::Mat ref = cv::imread(color_image_files[0], 0);
    Sophus::SE3d pose_ref_TWC = poses_TWC[0];
    double init_depth = 3.0;
    double init_cov2 = 3.0;
    cv::Mat depth(height, width, cv::CV_64F, init_depth);
    cv::Mat depth_cov(height, width, cv::CV_64F, init_cov2);

    for(int i = 1; i < color_image_files.size(); i++) {
        cv::Mat curr = cv::imread(color_image_files[i], 0);
        if (curr.data == nullptr) continue;
        Sophus::SE3d pose_curr_TWC = poses_TWC[i];
        Sophus::SE3d pose_T_C_R = pose_cur_TWC.inverse() * pose_ref_TWC;
        update(ref, curr, pose_T_C_R, depth, depth_cov2);
        evaluateDepth(ref_depth, depth);
        plotDepth(ref_depth, depth);
        imshow("image", curr);
        waitKey(1);
    }

    cv::imwrite("depth.png", depth);
    return 0;
}

bool readDatasetFiles(
    const string &path,
    std::vector<string> &color_image_files,
    std::vector<Sophus::SE3d> &poses,
    cv::Mat &ref_depth) {
        std::ifstream fin(path + "/first_200_frames_traj_over_table_input_sequence.txt");
        if (!fin) return false;

        while (!fin.eof()) {
            string image;
            fin >> image;
            double data[7];
            for (double &d:data) fin >> d;

            color_image_files.push_back(path + string("/images/") + image);
            poses.push_back(
                Sophus::SE3d(eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                    eigen::Vector3d(data[0], data[1], data[2]))
            );
            if (!fin.good()) break;
        }
        fin.close();

        fin.open(path + "/depthmaps/scene_000.depth");
        ref_depth = cv::Mat(height, width, cv::CV_64F);
        if (!fin) return false;
        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++) {
                double depth = 0;
                fin >> depth;
                ref_depth.ptr<double>(y)[x] = depth / 100.0;
            }

        return true;
}

bool update(
    const cv::Mat &ref, 
    const cv::Mat &curr, 
    const Sophus::SE3d &T_C_R, 
    cv::Mat &depth, 
    cv::Mat &depth_cov2) {
    for (int x = boarder; x < width - boarder; x++)
        for (int y = boarder; y < height - boarder; y++) {
            if (depth_cov2.ptr<double>(y)[x] < min_cov || depth_cov2.ptr<double>(y)[x] > max_cov) continue;

            eigen::Vector2d pt_curr;
            eigen::Vector2d epipolar_direction;
            bool ret = epipolarSearch(
                ref,
                curr,
                T_C_R,
                eigen::Vector2d(x, y),
                depth.ptr<double>(y)[x],
                sqrt(depth_cov2.ptr<double>(y)[x]),
                pt_curr,
                epipolar_direction
            );
            if (ret == false) continue;
            
            updateDepthFilter(eigen::Vector2d(x, y), pt_curr, T_C_R, epipolar_direction, depth, depth_cov2);
        }
}

bool epipolarSearch(
    const cv::Mat &ref, 
    const cv::Mat &curr,
    const Sophus::SE3d &T_C_R, 
    const eigen::Vector2d &pt_ref,
    const double &depth_mu, 
    const double &depth_cov,
    eigen::Vector2d &pt_curr, 
    eigen::Vector2d &epipolar_direction) {
    eigen::Vector3d f_ref = px2cam(pt_ref);
    f_ref.normalize();
    eigen::Vector3d P_ref = f_ref * depth_mu;

    eigen::Vector2d px_mean_curr = cam2px(T_C_R * P_ref);
    double d_min = depth_mu - 3 * depth_cov;
    double d_max = depth_mu + 3 * depth_cov;
    if (d_min < 0.1) d_min = 0.1;
    eigen::Vector2d px_min_curr = cam2px(T_C_R * (f_ref * d_min));
    eigen::Vector2d px_max_curr = cam2px(T_C_R * (f_ref * d_max)); 

    eigen::Vector2d epipolar_line = px_max_curr - px_min_curr;
    epipolar_direction = epipolar_line;
    epipolar_direction.normalize();
    double half_length = 0.5 * epipolar_line.norm();
    if (half_length > 100) half_length = 100;
    double best_ncc = -1.0;
    eigen::Vector2d best_px_curr;
    for (double l = -half_length; l <= half_length; l += 0.7) {
        eigen::Vector2d px_curr = px_mean_curr + l * epipolar_direction;
        if (!inside(px_curr)) continue;
        double ncc = NCC(ref, curr, pt_ref, px_curr);
        if (ncc > best_ncc) {
            best_ncc = ncc;
            best_px_curr = px_curr;
        }
    }
    if (best_ncc < 0.85f) return false;
    pt_curr = best_px_curr;
    return true;
}

double NCC(
    const cv::Mat &ref, 
    const cv::Mat &curr,
    const eigen::Vector2d &pt_ref, 
    const eigen::Vector2d &pt_curr) {
    double mean_ref = 0, mean_curr = 0;
    std::vector<double> values_ref, values_curr;
    for (int x = -ncc_window_size; x <= ncc_window_size; x++) {
        for (int y = -ncc_window_size; y <= ncc_window_size; y++) {
            double value_ref = double(ref.ptr<uchar>(int(y + pt_ref(1, 0)))[int(x + pt_ref(0, 0))]) / 255.0;
            mean_ref += value_ref;

            double value_curr = getBilinearInterpolatedValue(curr, pt_curr + std::Vector2d(x, y));
            mean_curr += value_curr;

            values_ref.push_back(value_ref);
            values_curr.push_back(value_curr);
        }
    }

    mean_ref /= ncc_area;
    mean_curr /= ncc_area;

    double numerator = 0, demoniator1 = 0, demoniator2 = 0;
    for (int i = 0; i < values_ref.size(); i++) {
        double n = (values_ref[i] - mean_ref) * (values_curr[i] - mean_curr);
        numerator += n;
        demoniator1 += (values_ref[i] - mean_ref) * (values_ref[i] - mean_ref);
        demoniator2 += (values_curr[i] - mean_curr) * (values_curr[i] - mean_curr);
    }
    return numerator / sqrt(demoniator1 * demoniator2 + 1e-10);
}

bool updateDepthFilter(
    const eigen::Vector2d &pt_ref,
    const eigen::Vector2d &pt_curr,
    const Sophus::SE3d &T_C_R,
    const eigen::Vector2d &epipolar_direction,
    cv::Mat &depth,
    cv::Mat &depth_cov2) {

    Sophus::SE3d T_R_C = T_C_R.inverse();
    eigen::Vector3d f_ref = px2cam(pt_ref);
    f_ref.normalize();
    eigen::Vector3d f_curr = px2cam(pt_curr);
    f_curr.normalize();

    eigen::Vector3d t = T_R_C.translation();
    eigen::Vector3d f2 = T_R_C.so3() * f_curr;
    eigen::Vector2d b = eigen::Vector2d(t.dot(f_ref), t.dot(f2));
    eigen::Matrix2d A;
    A(0, 0) = f_ref.dot(f_ref);
    A(0, 1) = -f_ref.dot(f2);
    A(1, 0) = -A(0, 1);
    A(1, 1) = -f2.dot(f2);
    eigen::Vector2d ans = A.inverse() * b;
    eigen::Vector3d xm = ans[0] * f_ref;
    eigen::Vector3d xn = t + ans[1] * f2;
    eigen::Vector3d p_esti = (xm + xn) / 2.0;
    double depth_estimation = p_esti.norm();   

    eigen::Vector3d p = f_ref * depth_estimation;
    eigen::Vector3d a = p - t;
    double t_norm = t.norm();
    double a_norm = a.norm();
    double alpha = acos(f_ref.dot(t) / t_norm);
    double beta = acos(-a.dot(t) / (a_norm * t_norm));
    eigen::Vector3d f_curr_prime = px2cam(pt_curr + epipolar_direction);
    f_curr_prime.normalize();
    double beta_prime = acos(f_curr_prime.dot(-t) / t_norm);
    double gamma = M_PI - alpha - beta_prime;
    double p_prime = t_norm * sin(beta_prime) / sin(gamma);
    double d_cov = p_prime - depth_estimation;
    double d_cov2 = d_cov * d_cov;

    double mu = depth.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))];
    double sigma2 = depth_cov2.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))];

    double mu_fuse = (d_cov2 * mu + sigma2 * depth_estimation) / (sigma2 + d_cov2);
    double sigma_fuse2 = (sigma2 * d_cov2) / (sigma2 + d_cov2);

    depth.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))] = mu_fuse;
    depth_cov2.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))] = sigma_fuse2;

    return true;
}

void plotDepth(const Mat &depth_truth, const Mat &depth_estimate) {
    imshow("depth_truth", depth_truth * 0.4);
    imshow("depth_estimate", depth_estimate * 0.4);
    imshow("depth_error", depth_truth - depth_estimate);
    waitKey(1);
}

void evaludateDepth(const Mat &depth_truth, const Mat &depth_estimate) {
    double ave_depth_error = 0;
    double ave_depth_error_sq = 0;
    int cnt_depth_data = 0;
    for (int y = boarder; y < depth_truth.rows - boarder; y++)
        for (int x = boarder; x < depth_truth.cols - boarder; x++) {
            double error = depth_truth.ptr<double>(y)[x] - depth_estimate.ptr<double>(y)[x];
            ave_depth_error += error;
            ave_depth_error_sq += error * error;
            cnt_depth_data++;
        }
    ave_depth_error /= cnt_depth_data;
    ave_depth_error_sq /= cnt_depth_data;

    cout << "Average squared error = " << ave_depth_error_sq << ", average error: " << ave_depth_error << endl;
}

void showEpipolarMatch(const Mat &ref, const Mat &curr, const Vector2d &px_ref, const Vector2d &px_curr) {
    Mat ref_show, curr_show;
    cv::cvtColor(ref, ref_show, CV_GRAY2BGR);
    cv::cvtColor(curr, curr_show, CV_GRAY2BGR);

    cv::circle(ref_show, cv::Point2f(px_ref(0, 0), px_ref(1, 0)), 5, cv::Scalar(0, 0, 250), 2);
    cv::circle(curr_show, cv::Point2f(px_curr(0, 0), px_curr(1, 0)), 5, cv::Scalar(0, 0, 250), 2);

    imshow("ref", ref_show);
    imshow("curr", curr_show);
    waitKey(1);
}

void showEpipolarLine(const Mat &ref, const Mat &curr, const Vector2d &px_ref, const Vector2d &px_min_curr,
                      const Vector2d &px_max_curr) {

    Mat ref_show, curr_show;
    cv::cvtColor(ref, ref_show, CV_GRAY2BGR);
    cv::cvtColor(curr, curr_show, CV_GRAY2BGR);

    cv::circle(ref_show, cv::Point2f(px_ref(0, 0), px_ref(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
    cv::circle(curr_show, cv::Point2f(px_min_curr(0, 0), px_min_curr(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
    cv::circle(curr_show, cv::Point2f(px_max_curr(0, 0), px_max_curr(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
    cv::line(curr_show, Point2f(px_min_curr(0, 0), px_min_curr(1, 0)), Point2f(px_max_curr(0, 0), px_max_curr(1, 0)),
             Scalar(0, 255, 0), 1);

    imshow("ref", ref_show);
    imshow("curr", curr_show);
    waitKey(1);
}