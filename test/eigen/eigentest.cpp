#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;

void print_matrix(const Matrix<float, Dynamic, Dynamic> &mat);

int main(int argc, char **argv) {

    Matrix3d rotation_matrix = Matrix3d::Identity();
    AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));
    cout << "rotation matrix is \n" << rotation_vector.matrix() << endl;

    rotation_matrix = rotation_vector.toRotationMatrix();
    
}

void print_matrix(const Matrix<float, Dynamic, Dynamic> &mat) {

    for(int i = 0; i < mat.rows(); i++){
        for(int j = 0; j < mat.cols(); j++) {
            cout << mat(i, j) << "\t";
        }
        cout << endl;
    }
}
