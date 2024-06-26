#include <iostream>
#include <ceres/ceres.h>
#include "BALProblem.h"

void SolveBA();

int main(int argc, char **argv) {

    BALProblem bal_problem(argv[1]);
    bal_problem.Normalize();
    bal_problem.Perturb(0.1, 0.5, 0.5);
    bal_problem.WriteToPLYFile("initial.ply");
    SolveBA(bal_problem);
    bal_problem.WriteToPLYFILE("final.ply");

    return 0;
}

void SolveBA(BALProblem &bal_problem) {
    const int point_block_size = bal_problem.point_block_size();
    const int camera_block_size = bal_problem.camera_block_size();
    double *points = bal_problem.mutable_points();
    double *cameras = bal_problem.mutable_cameras();

    const double *observations = bal_problem.observations();
    ceres::Problem problem;

    for(int i = 0; i < bal_problem.num_observations(); i++) {
        ceres::CostFunction *cost_function;
        cost_function = SnavelyReprojectionError::Create(observations[2 * i], observations[2 * i + 1]);
        ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);

        double *camera = cameras + camera_block_size * bal_problem.camera_index()[i];
        double *point = points + point_block_size * bal_problem.point_index()[i];

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