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


class Detector {
public:
    Detector(cv::Mat& camera_frame, std::shared_mutex& mutex);                 // Default constructor
    Detector(int nfeatures, int nOctaveLayers, double contrastThreshold,
             double edgeThreshold, double sigma, bool enable_precise_upscale,
             cv::Mat& camera_frame, std::shared_mutex& mutex);
    Detector(size_t max_keypoints, cv::Mat& camera_frame, std::shared_mutex& mutex);
    ~Detector();                // Destructor


    //used in a thread
    void DetectCompute(cv::Mat mask,
                       CV_OUT std::vector < cv::KeyPoint > & keypoints,
                       cv::Mat descriptors,
                       bool useProvidedKeypoints = false);

    void SetMatForCameraSL(cv::Mat& camera_frame);
    void SetNewFrameReadyFlag();
private:
    cv::Ptr<cv::SIFT> m_detector;
    std::vector<AssemblyPart> components;
    std::shared_mutex&m_mutex;
    bool m_new_frame_rq;
    //sl::Mat& m_camera_frame_SL;        //shared between main-camera-detector threads
    //cv::Mat m_camera_frame_MAT;     //local for the detector thread, from m_camera_frame
    cv::Mat& m_camera_frame_MAT;        //shared between main-camera-detector threads


};