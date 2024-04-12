#pragma once

#include <sl/Camera.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <shared_mutex> 
#include <mutex> 
#include <thread> 
#include <chrono>
#include "Enums.hpp"

class AugmentedInstructions {
public:
    AugmentedInstructions(std::vector<std::vector<cv::Point>>& corners, cv::Mat& image, std::mutex& mutex);
    ~AugmentedInstructions();                // Destructor

    void StartInstructions();

private:

    unsigned m_parts_cnt = 0;
    std::mutex& m_mutex;                                      //mutexes used for notifying the main thread that new object locations have been updated

    std::vector<std::vector<cv::Point>>& m_corners;
    cv::Mat& m_image;
    cv::Mat m_image_clean;

    void DrawCorners();
};