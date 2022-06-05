#include <iostream>
#include <opencv2/core/core.hpp>
#include "sophos/se3.hpp"

using namespace std;
using namespace cv;

void bundleAdjustomentGaussNewton(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose)
{
    typedef Eigen::Matrix<double, 6, 1> Mat61;
    const int iterations = 100;
    double cost = 0, lastCost = 0;
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    for (int iter = 0; iter < iterations; iter++)
    {
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;

        for (int i = 0; i < points_3d.size(); i++)
        {
            Eigen::Vector3d pc = pose * points_3d[i];
            double inv_z = 1.0 / pc[2];
            double inv_z2 = inv_z * inv_z;
            Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);
            Eigen::Vector2d e = points_2d[i] - proj;
            cost += e.squaredNorm();
            Eigen::Matrix<double, 2, 6> J;
            J << -fx * inv_z, 0, fx * pc[0] * inv_z2, -fx - fx * pc[0] * pc[0] * inv_z2,
                fx * pc[1] * inv_z, 0, -fy * inv_z, fy * pc[1] * inv_z2,
                fy + fy * pc[1] * pc[1] * inv_z2, -fy * pc[0] * inv_z;
            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

        Vector6d dx;
        dx = H.ldlt().solve(b);

        if (isnan(dx[0]))
        {
            cout << "dx is nan" << endl;
            break;
        }

        if (iter > 0 && cost < lastCost)
        {
            cout << "cost: " << cost << endl
                 << "last cost: " << lastCost << endl;
            break;
        }

        post = Sophus::SE3d::exp(dx) * pose;
        lastCost = cost;

        cout << "iterations" << iter << "cost: " << cost << endl;
        if (dx.norm() < 1e-6)
        {
            break;
        }
    }
}

cout << "pose: " << pose.matrix() << endl;
}

int main(int argc, char **argv)
{
    Mat r, t;
    solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false);

    Mat R;
    cv::Rodrigues(r, R);

    cout << "R: " << endl
         << R << endl;
    cout << "t: " << endl
         << t << endl;
}