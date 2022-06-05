#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>

using namespace std;
using namespace cv;

void pose_estimation_2d2d(std::vector<key_point> key_points_1,
                          std::vector<key_point> key_points_2,
                          std::vectpr<DMatch> matches,
                          Mat &R, Mat &t)
{
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    vector<Point2f> points1;
    vector<Point2f> points2;

    for (int i = 0; i < (int)matches.size(); i++)
    {
        point1.push_back(key_points_1[matches[i].queryIdx].pt);
        point2.push_back(key_points_2[matches[i].trainIdx].pt);
    }

    Mat fundamental_matrix = findFundamentalMat(points1, points2, CV_FM_8POINT);

    cout << "fundamental_matrix = \n"
         << fundamental_matrix << endl;

    Point2d prinicipal_point(325.1, 249.7);

    double focal_length = 521;

    Mat essential_matrix = findEssentialMat(points1, points2, focal_length, prinicipal_point);

    cout << "essential_matrix: " << endl
         << essential_matrix << endl;

    Mat homography_matrix = findHomography(points1, points2, RANSAC, 3);
    cout << "homography_matrix: " << endl
         << homography_matrix << endl;

    recoverPose(essential_matrix, points1, points2, R, t, focal_length, prinicipal_point);

    cout << "R: " << endl
         << R << endl;
    cout << "t: " << endl
         << t << endl;
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cout << "usage: pose_estimation_2d2d img1 img2" << endl;
        return 1;
    }

    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

    assert(img_1.data && img_2.data && "Could not load images");

    vector<key_point> key_points_1, key_points_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, key_points_1, key_points_2, matches);
    cout << "In total, we get" << matches.size() << "matches" << endl;

    Mat R, t;
    pose_estimation_2d2d(key_points_1, key_points_2, matches, R, t);

    Mat t_x = (Mat_<double>(3, 3) << 1, 0, 0,
               0, 1, 0,
               t.at<double>(0, 0), t.at<double>(1, 0), 1);

    cout << "t^R=" << endl
         << t_x * R << endl;

    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    for (DMatch m : matches)
    {
        Point2d pt1 = pixel2cam(key_points_1[m.queryIdx].pt, K);
        Point2d pt2 = pixel2cam(key_points_2[m.trainIdx].pt, K);
        Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
        Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
        Mat d = y2.t() * t_x * R * y1;
        cout << "epipolar constraint = " << d << endl;
    }
    return 0;
}
