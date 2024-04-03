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

class AugmentedInstructions {
public:
    AugmentedInstructions(std::vector<std::vector<cv::Point>>& corners, cv::Mat& image);
    ~AugmentedInstructions();                // Destructor


    void StartBlinkTimer();

    void BlinkPlanes();
private:

    std::chrono::milliseconds m_delay;
    std::thread m_thread;
    std::atomic<bool> m_blink;

    std::vector<std::vector<cv::Point>>& m_corners;
    cv::Mat& m_image;
    cv::Mat m_image_clean;

    void BlinkPlanes(bool colour);
};
