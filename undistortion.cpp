#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

string image_file = "./dostorted.png";

int main(int argc, char **argv)
{
    // rad-tan model prams
    double k1 = -0.2, k2 = 0.1, p1 = 0.1, p2 = -0.1;
    // intrinsics
    double fx = 1.0, fy = 1.0, cx = 0.5, cy = 0.5;

    cv::Mat image = cv::imread(image_file, 0);
    int rows = image.rows, cols = image.cols;
    cv::image_undistort = cv::Mat(rows, cols, CV_8UC1);

    for (int v = 0; v < rows; v++)
    {
        for (int u = 0; u < cols; u++)
        {
            double x = (u - cx) / fx, y = (v - cy) / fy;
            double r = sqrt(x * x + y * y);
            double x_undistorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
            double y_undistorted = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
            double u_undistorted = fx * x_undistorted + cx;
            double v_undistorted = fy * y_undistorted + cy;

            if (u_undistorted >= 0 && v_undistorted >= 0 && u_undistorted < cols && v_undistorted < rows)
            {
                image_undistort.at<uchar>(v, u) = image.at<uchar>(v_undistorted, u_undistorted);
            }
            else
            {
                image_undistort.at<uchar>(v, u) = 0;
            }
        }
    }

    cv::imshow("image_undistort", image_undistort);
    cv::waitKey(0);
    return 0;
}