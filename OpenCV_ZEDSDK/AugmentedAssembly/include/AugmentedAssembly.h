#pragma once

#include "CameraHandler.h"
#include "Detector.h"
#include "Enums.hpp"
#include "AssemblyPart.h"
#include "AugmentedInstructions.h"
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

constexpr int PRINT_AA = 0;

class AugmentedAssembly {

public:
    __declspec(dllexport) AugmentedAssembly();                    // Default constructor
    __declspec(dllexport) ~AugmentedAssembly();                   // Destructor

    /**
     * @brief Returns the left augmented frame as a char array
     * 
     * @return char array of the left augmented image
     */
    __declspec(dllexport) char* GetLeftFrame();

    /**
     * @brief Returns the right augmented frame as a char array
     *
     * @return char array of the right augmented image
     */
    __declspec(dllexport) char* GetRightFrame();

    /**
     * @brief Starts the whole application
     * 
     */
    __declspec(dllexport) void Start();

private:
    int m_keypoints_size = 0;

    const Method m_method = Method::BRISK;
    unsigned m_parts_cnt = 0;

    AssemblyStates m_assembly_state = AssemblyStates::AssemblyStart;
    unsigned m_steps_cnt = 0;
    std::vector<unsigned> m_step_indices;

    // Private member variables
    sl::Mat m_grabbed_frame_left_SL;
    sl::Mat m_grabbed_frame_right_SL;
    cv::Mat m_grabbed_frame_left_MAT;
    cv::Mat m_grabbed_frame_right_MAT;
    cv::Mat m_detector_frame_MAT_new;

    CameraHandler m_zed;
    Detector m_detector;
    AugmentedInstructions m_instructions;
    std::vector<cv::DMatch> best_good_matches_filtered;
    std::vector<std::vector<std::vector<cv::Point>>> m_scene_corners;

    /**
     * @brief Based on the number of folders, calculates the number of parts
     * 
     * @param path_to_parts Absolute path to the objects' folders
     */
    void GetNumberOfParts(std::filesystem::path path_to_parts);
    std::vector<AssemblyPart> m_assembly_parts;
    std::vector<std::thread> m_threads_parts;
    std::unique_ptr < std::vector<std::mutex>> m_mutex_parts;                  
    std::unique_ptr<std::vector<uint8_t>> m_parts_new_rq;                       //for notifying the object threads to start Matching
    std::unique_ptr < std::vector<std::condition_variable_any>> m_cv_parts;


    std::thread thread_camera;
    std::thread thread_detector;
    std::thread m_thread_instructions;

    std::vector<cv::KeyPoint> m_keypoints_scene;
    cv::Mat m_descriptor_scene;

    std::shared_mutex m_mutex;
    std::mutex m_mutex_detector;
    std::atomic<bool> m_detector_new_frame_rq;


    // Mapping between sl::Mat and cv::Mat
    inline cv::Mat slMat2cvMat(sl::Mat& input)
    {
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
            return cv::Mat(static_cast<int>(input.getHeight()), static_cast<int>(input.getWidth()), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
        }
        return ret;
    }


};
