#pragma once

#include <sl/Camera.hpp>

#include <shared_mutex> 
#include <mutex> 
#include <thread> 


class CameraHandler {
public:
    //CameraHandler();                 // Default constructor
    CameraHandler(sl::Mat& detector_frame, std::shared_mutex &mutex);
    ~CameraHandler();                // Destructor
    
    sl::ERROR_CODE GetCameraState();

    //used in a thread
    void Start();

    void SetMatForDetectorSL(sl::Mat& detector_frame);
    void SetWantNewFrameFlag();


private:
    sl::Camera m_zed;
    bool m_failure;
    sl::ERROR_CODE m_returned_state;
    sl::Mat m_grabbed_frame;

    bool m_new_frame_rq;
    std::shared_mutex& m_mutex;
    sl::Mat &m_detector_frame;



};