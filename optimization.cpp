#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophos/se3.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();

    Quaterniond q(R);
    Sophus::SO3d SO3_R(R);
    Sophus::SO3d SO3_q(q);

    cout << SO3_R.matrix() << endl;
    cout << SO3_q.matrix() << endl;

    Vector3d so3 = SO3_R.log();
    cout << "so3: " << so3.transpose() << endl;

    cout << Sophus::SO3d::hat(so3) << endl;

    cout << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

    Vector3d update_so3(1e-4, 0, 0);

    Sophus::SO3d SO3_R_update = Sophus::SO3d::exp(update_so3) * SO3_R;

    cout << SO3_R_update.matrix() << endl;

    // SE3

    Vector3d t(1, 0, 0);

    Sophus::SE3d SE3_R_t(R, t);
    Sophus::SE3d SE3_q_t(q, t);

    cout << "SE3_R_t: " << endl
         << SE3_R_t.matrix() << endl;

    cout << "SE3_q_t: " << endl

        typedef Eigen::Matrix<double, 6, 1>
            Vector6d;

    Vector6d se3 = SE3_R_t.log();

    cout << "se3: " << se3.transpose() << endl;

    cout << "se3 hat =" << endl
         << Sophus::SE3d::hat(se3) << endl;

    cout << "se3 vee =" << endl
         << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)) << endl;

    Vector6d update_se3(1e-4, 0, 0, 0, 0, 0);

    Sophus::SE3d SE3_R_t_update = Sophus::SE3d::exp(update_se3) * SE3_R_t;

    cout << "SE3_R_t_update: " << endl
         << SE3_R_t_update.matrix() << endl;
}