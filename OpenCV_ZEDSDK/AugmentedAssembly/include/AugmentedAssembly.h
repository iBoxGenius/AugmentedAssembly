#pragma once

#include "CameraHandler.h"
#include "Detector.h"
#include "Enums.hpp"
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

class AugmentedAssembly {

public:
    AugmentedAssembly();                    // Default constructor
    ~AugmentedAssembly();                   // Destructor

    void Start();

private:
    // Private member variables
    sl::Mat m_grabbed_frame_SL;
    cv::Mat m_grabbed_frame_MAT;
    cv::Mat m_detector_frame_MAT;

    CameraHandler m_zed;
    Detector m_detector;
    /*
    * Each assembly part will have its own matcher
    * std::vector<AssemblyPart> assembly_parts;     -> each has an instance of Matcher class
    *                                               
    * AssemblyPart has a method which starts the matching process ==> thread function -> comparing based on the number of descriptors
    *       AugmentedAssembly fires up the threads (by batches of x (== 4 ?))       //same handling logic as the Detector class, in terms of mutexes and atomic<bool>, cv::Mat reference
    */

    std::thread thread_camera;
    std::thread thread_detector;

    std::vector<cv::KeyPoint> m_keypoints_scene;
    cv::Mat m_descriptor_scene;
    Method m_method = Method::BRISK;

    std::shared_mutex m_mutex;
    std::mutex m_mutex_detector;
    std::atomic<bool> m_detector_new_frame_rq;


    // Mapping between sl::Mat and cv::Mat
    inline cv::Mat slMat2cvMat(sl::Mat& input) {
        int cv_type = -1;
        cv::Mat ret;
        switch(input.getDataType()) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1;
            break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2;
            break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3;
            break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4;
            break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1;
            break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2;
            break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3;
            break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4;
            break;
        default: break;
        }

        if(input.getPtr<sl::uchar1>(sl::MEM::CPU) != NULL)
        {
            return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
        }
        return ret;
    }


};
