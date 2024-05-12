#pragma once

#include <sl/Camera.hpp>

#include <shared_mutex> 
#include <mutex> 
#include <thread> 


class CameraHandler {
public:
    //CameraHandler();                 // Default constructor
    CameraHandler(sl::Mat& camera_frame_left, sl::Mat& camera_frame_right, std::shared_mutex &mutex);
    ~CameraHandler();                // Destructor
    
    sl::ERROR_CODE GetCameraState();

    //used in a thread
    void StartCamera();

    void SetMatForDetectorSL(sl::Mat& camera_frame);
    void SetWantNewFrameFlag();

private:
    sl::Camera m_zed;
    bool m_failure;
    sl::ERROR_CODE m_returned_state;
    sl::Mat m_grabbed_frame_left;
    sl::Mat m_grabbed_frame_right;

    std::shared_mutex& m_mutex;
    sl::Mat &m_camera_frame_ref_left;
    sl::Mat& m_camera_frame_ref_right;

    void InitCamera(sl::Camera &zed);

};