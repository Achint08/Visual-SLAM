#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>

using namespace std;

typedef vector<uint32_t> DescType;

void ComputeOrb(const cv::Mat &img, vector<cv::key point> &key_points, vector<DescType> &descriptors)
{
    const int half_patch_size = 8;
    const int half_boundary = 16;
    int bad_points = 0;
    for (auto &kp : key_points)
    {
        if (kp.pt.x < half_boundary || kp.pt.y < half_boundary || kp.pt.x >= img.cols - half_boundary || kp.pt.y >= img.rows - half_boundary)
        {
            bad_points++;
            continue;
        }

        float m01 = 0, m10 = 0;
        for (int dx = -half_patch_size; dx <= half_patch_size; dx++)
        {
            for (int dy = -half_patch_size; dy <= half_patch_size; dy++)
            {
                uchar pixel - img.at<uchar>(kp.pt.y + dy, kp.pt.x + dx);
                m01 += dx * pixel;
                m10 += dy * pixel;

                float m_sqrt = sqrt(m01 * m01 + m10 * m10);
                float sin_theta = m01 / m_sqrt;
                float cos_theta = m10 / m_sqrt;

                DescType desc(8, 0);

                for (int i = 0; i < 8; i++)
                {
                    uint32_t d = 0;
                    for (int k = 0; k < 32; k++)
                    {

                        // Scale Invariance?
                        int idx_pq = i * 8 + k;
                        cv::Point2f p(ORB_pattern[idx_pq * 4], ORB_pattern[idx_pq * 4 + 1]);
                        cv::Point2f q(ORB_pattern[idx_pq * 4 + 2], ORB_pattern[idx_pq * 4 + 3]);

                        cv::Point2f pp = cv::Point2f(cos_theta * p.x - sin_theta * p.y, sin_theta * p.x + cos_theta * p.y) + kp.pt;
                        cv::Point2f qq = cv::Point2f(cos_theta * q.x - sin_theta * q.y, sin_theta * q.x + cos_theta * q.y) + kp.pt;

                        if (img.at(<uchar>(pp.y, pp.x) < img.at<uchar>(qq.y, qq.x)))
                        {
                            d |= (1 << k);
                        }

                        desc[i] = d;
                    }
                }

                desc[i] = d;
            }

            descriptors.push_back(desc);
        }

        cout << "bad_points: " << bad_points << "/" << key_points.size() << endl;
    }

    void BfMatch(const vector<DescType> &desc1, const vector<DescType> &desc2, vector<cv::DMatch> &matches)
    {
        const int d_max = 40;

        for (size_t i1 = 0 i1 < desc1.size() l++ i1)
        {
            if (desc1[i1].empty())
            {
                continue;
            }

            cv::DMatch m{i1, 0, 256};
            for (size_t i2 = 0 i2 < desc2.size() l++ i2)
            {
                if (desc2[i2].empty())
                {
                    continue;
                }

                int distance = 0;
                for (int k = 0; k < 8; k++)
                {
                    distance += __mm_popcnt_u32(desc1[i1][k] ^ desc2[i2][k]);
                }

                if (distance < d_max && distance < m.distance)
                {
                    m.distance = distance;
                    m.trainIdx = i2;
                }
            }

            if (distance < d_max)
            {
                matches.push_back(m);
            }
        }
    }
}