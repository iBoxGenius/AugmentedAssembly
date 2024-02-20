#pragma once

#include <sl/Camera.hpp>

#include <shared_mutex> 
#include <mutex> 
#include <thread> 


class CameraHandler {
public:
    //CameraHandler();                 // Default constructor
    CameraHandler(sl::Mat& camera_frame, std::shared_mutex &mutex);
    ~CameraHandler();                // Destructor
    
    sl::ERROR_CODE GetCameraState();

    //used in a thread
    void Start();

    void SetMatForDetectorSL(sl::Mat& camera_frame);
    void SetWantNewFrameFlag();

private:
    sl::Camera m_zed;
    bool m_failure;
    sl::ERROR_CODE m_returned_state;
    sl::Mat m_grabbed_frame;

    std::shared_mutex& m_mutex;
    sl::Mat &m_camera_frame_ref;

    void InitCamera(sl::Camera &zed);

};