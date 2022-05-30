#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pangolin/pangolin.h>
#include <string>

using namespace std;

int main(int argc, char **argv)
{
    // intrinsics
    double fx = 1.0, fy = 1.0, cx = 0.5, cy = 0.5;

    double b = 0.573;

    cv::Mat left = cv::imread(left_file, 0);
    cv::Mat right = cv::imread(right_file, 0);
    // Semi global block matching
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);

    cv::Mat disparity_sgbm, disparity;

    sgbm->compute(left, right, disparity_sgbm);
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

    vector<Vector4d, Eigen::aligned_allocator<Vector4d>> point_cloud;

    for (int v = 0; v < left.rows; v++)
    {
        for (int u = 0; u < left.cols; u++)
        {
            if (disparity.at<float>(v, u) <= 10.0 || disparity.at<float>(v, u) > 96.0)
                continue;
            Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0);

            double x = (u - cx) / fx, y = (v - cy) / fy;
            double depth = fx * b / disparity.at<float>(v, u);
            point[0] = x * depth;
            point[1] = y * depth;
            point[2] = depth;

            pointcloud.push_back(point);
        }
    }

    cv.imshow("disparity", disparity / 96.0);

    cv.waitKey(0);

    showPointCloud(pointcloud, "point cloud");
    return 0;
}