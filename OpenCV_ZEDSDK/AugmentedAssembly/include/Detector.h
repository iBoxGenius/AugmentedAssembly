#pragma once

#include "AssemblyPart.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <shared_mutex> 
#include <mutex> 
#include <thread> 
#include <string>

class Detector {
public:
    Detector(const std::string type, cv::Mat & camera_frame, std::mutex& mutex, std::atomic<bool>& sync_var);
    ~Detector();                // Destructor

    //used in a thread
    void DetectCompute(cv::Mat mask, CV_OUT std::vector < cv::KeyPoint > & keypoints, cv::Mat& descriptors);

    void SetMatForCameraSL(cv::Mat& camera_frame);

private:
    std::string m_type;
    cv::Ptr<cv::SIFT> m_detector_sift;
    cv::Ptr<cv::ORB> m_detector_orb;
    cv::Ptr<cv::BRISK> m_detector_brisk;

    std::vector<AssemblyPart> components;
    std::mutex &m_mutex;
    std::atomic<bool>& m_new_frame_rq;
    //sl::Mat& m_camera_frame_SL;        //shared between main-camera-detector threads
    //cv::Mat m_camera_frame_MAT;     //local for the detector thread, from m_camera_frame
    cv::Mat& m_camera_frame_MAT_ref;        //shared between main-camera-detector threads


};