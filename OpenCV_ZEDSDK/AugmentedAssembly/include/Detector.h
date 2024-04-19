#pragma once

#include "AssemblyPart.h"
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

#include <Windows.h>

constexpr size_t KP_MAX = 2000;
constexpr double KP_RETAIN = 0.9;
constexpr int KP_RETAIN_BODER_SIZE = 20;

class Detector {
public:
    Detector(Method method, cv::Mat & camera_frame, std::mutex& mutex, std::atomic<bool>& sync_var);
    //Detector(Method method, cv::Mat& camera_frame);
    ~Detector();                // Destructor

    //used in a thread
    void DetectCompute(cv::Mat mask, CV_OUT std::vector < cv::KeyPoint > & keypoints, cv::Mat& descriptors);

    void SetMatForCameraSL(cv::Mat& camera_frame);
    int GetDescriptorSize();

private:
    //std::string m_type;
    cv::Ptr<cv::SIFT> m_detector_sift;
    cv::Ptr<cv::ORB> m_detector_orb;
    cv::Ptr<cv::BRISK> m_detector_brisk;
    Method m_method;

    std::vector<AssemblyPart> components;
    std::mutex &m_mutex;
    std::atomic<bool>& m_new_frame_rq;
    //sl::Mat& m_camera_frame_SL;        //shared between main-camera-detector threads
    //cv::Mat m_camera_frame_MAT;     //local for the detector thread, from m_camera_frame
    cv::Mat& m_camera_frame_MAT_ref;        //shared between main-camera-detector threads


};