#include "slam/backend.h"

namespace slam {

Backend::Backend() {
    point_block_size = bal_problem.point_block_size();
    camera_block_size = bal_problem.camera_block_size();
    points = bal_problem.mutable_points();
    cameras = bal_problem.mutable_cameras();
}

Backend::BundleAdjustment() {
    
    const double *observations = bal_problem.observations();
    ceres::Problem problem;

    for(int i = 0; i < bal_problem.num_observations(); i++) {
        cost_function = SnavelyReprojectionError::Create(observations[2 * i], observations[2 * i + 1]);
        loss_function = new ceres::HuberLoss(1.0);

        camera = cameras + camera_block_size * bal_problem.camera_index()[i];
        point = points + point_block_size * bal_problem.point_index()[i];

        problem.AddResidualBlock(cost_function, loss_function, camera, point);
    }
    std::cout << "Num Cameras: " << bal_problem.num_cameras() << " Number points: " << bal_problem.num_points() << " points." << std::endl;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::LinearSolverType::SPACE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
}

}