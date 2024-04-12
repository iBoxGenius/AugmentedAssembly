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

    void BlinkPlanesWin();
    bool m_blink_win = true;
private:

    std::chrono::milliseconds m_delay;
    std::thread m_thread;
    std::atomic<bool> m_blink;

    std::vector<std::vector<cv::Point>>& m_corners;
    /*
    cv::Mat& m_image;
    cv::Mat m_image_clean;
    */
    cv::Mat m_image_draw;
    cv::Mat& m_image_clean;

    std::atomic<bool> m_blink_rq;
    bool m_blink_img_first = true;
    unsigned m_blink_img_cnt = 0;
    cv::Mat image_clean_blink;

    void BlinkPlanes(bool colour);
};
