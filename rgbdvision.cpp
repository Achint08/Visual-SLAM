#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pangolin/pangolin.h>
#include <string>
#include "sophos/se3.hpp"

using namespace std;

int main(int argc, char **argv)
{
    vector<cv::Mat> colorImgs, depthImgs;

    TrejectoryType poses; // camera pose

    ifstream fin("./pose.txt");

    if (!fin)
    {
        cerr << "Failed to open file " << endl;
        return -1;
    }

    for (int i = 0; i < 5; i++)
    { // five images
        boost::format fmt("%s/%d.%s");
        colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
        depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "pgm").str(), -1));

        double data[7] = {0};
        for (auto &d : data)
            fin >> d;
        Sophus::SE3d pose(Eigen::Quaterniond(data[6], data[3], data[4], data[5])), Eigen::Vector3d(data[0], data[1], data[2]);
        poses.push_back(pose);

        doube cx = 325.5;
        double cy = 253.5;
        double fx = 518.0;
        double fy = 519.0;
        double depthScale = 1000.0;
        vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
        pointcloid.reserve(1000000);

        for (int i = 0; i < 5; i++)
        {
            cout << "Converting RGBD images to pointcloud..." << i + 1 << endl;
            cv::Mat color = colorImgs[i];
            cv::Mat depth = depthImgs[i];
            Sophus::SE3d T = poses[i];

            for (int v = 0; v < color.rows; v++)
            {
                for (int u = 0; u < color.cols; u++)
                {
                    unsigned int d = depth.ptr<unsigned short>(v)[u];
                    if (d == 0)
                        continue;
                    Eigen::Vector3d point;
                    point[2] = double(d) / depthScale;
                    point[0] = (u - cx) * point[2] / fx;
                    point[1] = (v - cy) * point[2] / fy;
                    Eigen::Vector3d pointWorld = T * point;

                    Vector6d p;
                    p.head<3>() = pointWorld;
                    p[5] = color.data[v * color.step + u * color.channels()];     // blue
                    p[4] = color.data[v * color.step + u * color.channels() + 1]; // green
                    p[3] = color.data[v * color.step + u * color.channels() + 2]; // red
                    pointcloud.push_back(p);
                }
            }
        }

        cout << "global point cloud has " << pointcloud.size() << " points." << endl;
        showPointCloud(pointcloud, "point cloud");
        return 0;
    }
}