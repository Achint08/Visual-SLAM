#include <iostream>
#include <chrono>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
    cv::Mat image;
    image = cv::imread(argv[1]);

    if (image.data == nullptr)
    {
        cerr << "file" << argv[1] << "not found" << endl;
        return 0;
    }

    cout << "Image cols: " << image.cols << "rows: " << image.rows << "channels: " << image.channels() << endl;

    cv::imshow("image", image);
    cv::waitKey(0);

    if (image.type() != CV_8UC1 && image.type() != CV_8UC3)
    {
        cout << "image type incorrect" << endl;
        return 0;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now() for (size_t y = 0; y < image.rows; y++)
    {
        unsigned char *row_ptr = image.ptr<unsigned char>(y);
        for (size_t x = 0; x < image.cols; x++)
        {
            unsigned char *data_ptr = &row_ptr[x * image.channels()];
            for (int c = 0; c != image.channels; c++)
            {
                unsigned char data = data_ptr[c];
            }
        }
    }

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_duration = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    cout << "time: " << time_duration.count() << "seconds" << endl;

    cv::Mat image_another = image;

    image_another(cv::Rect(0, 0, 100, 100)).setTo(0);

    cv.imshow("image_another", image_another);
    cv::waitKey(0);

cv:
    Mat image_clone = image.clone();
    image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
    cv::imshow("image_clone", image_clone);
    cv::waitKey(0);

    cv::destroyAllWindows();
    return 0;
}