#include <iostream>
using namespace std;

#include <ctime>

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

int main(int argc, char **argv)
{
    Matrix<float, 2, 3> matrix_23;

    Vector3d v_3d;
    Matrix<float, 3, 1> vd_3d;

    Matrix3d Matrix_33 = Matrix3d::Zero();

    Matrix<double, Dynamic, Dynamic> matrix_dynamic;

    MatrixXd matrix_x;

    matrix_23 << 1, 2, 3, 4, 5, 6;

    cout << "matrix_23: " << endl
         << matrix_23 << endl;

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            cout << matrix_23(i, j) << "\t";
            cout << endl;
            ;
        }
    }

    v_3d << 3, 2, 1;
    vd_3d << 4, 5, 6;

    Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    cout << "result: " << endl
         << result << endl;

    Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
    cout << "result2: " << endl
         << result2 << endl;

    Matrix_33 = Matrix3d::Random();

    cout << "Matrix_33: " << endl
         << Matrix_33 << endl;
    cout << "Transpose:" << endl
         << Matrix_33.transpose() << endl;

    cout << "Sum:" << endl
         << Matrix_33.sum() << endl;

    cout << "Trace:" << endl
         << Matrix_33.trace() << endl;

    SelfAdjointEigenSolver<Matrix3d> eigensolver(Matrix_33.transpose() * Matrix_33);

    cout << "Eigenvalues:" << endl
         << eigensolver.eigenvalues() << endl;
    cout << "Eigenvectors:" << endl
         << eigensolver.eigenvectors() << endl;
}