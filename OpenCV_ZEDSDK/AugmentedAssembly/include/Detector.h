#pragma once
#include "Enums.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <shared_mutex> 
#include <mutex> 
#include <thread> 
#include <string>
#include <iostream>

#include <Windows.h>

constexpr int PRINT_DET = 0;

//constexpr size_t KP_MAX = 2500;
constexpr size_t KP_MAX = 1000;
constexpr double KP_RETAIN = 0.9;
constexpr int KP_RETAIN_BODER_SIZE = 20;

class Detector {
public:

    /**
     * @brief Constructor
     * 
     * \param method Method for the descriptor/keypoints
     * \param camera_frame Frame to analyse
     * \param mutex Mutex to protect sync_var
     * \param sync_var Serves as request variable
     */
    Detector(Method method, cv::Mat & camera_frame, std::mutex& mutex, std::atomic<bool>& sync_var);
    ~Detector();
    //used in a thread
    /**
     * Runs in a thread. Detects and computes the descriptor + keypoints
     * 
     * \param mask Unused
     * \param keypoints [out]
     * \param descriptors [out]
     */
    void DetectCompute(cv::Mat mask, CV_OUT std::vector < cv::KeyPoint > & keypoints, cv::Mat& descriptors);

    /**
     * @brief Returns the size of the descriptor (width)
     * 
     * \return 
     */
    int GetDescriptorSize();

private:
    cv::Ptr<cv::BRISK> m_detector_brisk;
    Method m_method;
    std::mutex &m_mutex;
    std::atomic<bool>& m_new_frame_rq;
    cv::Mat& m_camera_frame_MAT_ref;        //shared between main-camera-detector threads
};