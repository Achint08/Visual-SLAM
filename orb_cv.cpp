#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{

    if (argc != 3)
    {
        cout << "Usage: orb_cv <image1> <image2>" << endl;
    }

    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    assert(img_1.data != nullptr && img_2.data != nullptr);

    std::vector<Keypoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;

    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    chrono::steady_clock::time_point begin = chrono::steady_clock::now();
    detector->detect(img_1, keypoints_1);
    descriptor->compute(img_1, keypoints_1, descriptors_1);

    detector->detect(img_2, keypoints_2);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(end - begin);
    cout << "Time taken by ORB: " << time_span.count() << " seconds." << endl;

    drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("ORB Features", outimg1);

    vector<DMatch> matches;
    t1 = chrono::steady_clock::now();
    matcher->match(descriptors_1, descriptors_2, matches);
    t2 = chrono::steady_clock::now();
    chrono::duration<double> time_span2 = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "Time taken by matcher: " << time_span2.count() << " seconds." << endl;

    auto min_max = minmax_element(mathces.begin(), matches.end(), [](const DMatch &m1, const DMatch &m2)
                                  { return m1.distance < m2.distance; });

    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    std::vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (matches[i].distance <= max(2 * min_dist, 30.0))
        {
            good_matches.push_back(matches[i]);
        }
    }

    Mat img_matches;
    Mat img_goodmatch;
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::DRAW_OVER_OUTIMG);
    imshow("All matches", img_matches);
    imshow("Good matches", img_goodmatch);
    waitkey(0);

    return 0;
}