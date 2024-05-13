#pragma once

#include <sl/Camera.hpp>

#include <shared_mutex> 
#include <mutex> 
#include <thread> 


class CameraHandler {
public:
    /**
     * @brief Constructor
     * 
     * @param camera_frame_left Reference to the left camera frame
     * @param camera_frame_right Reference to the right camera frame
     * @param mutex Mutex to protect both
     */
    CameraHandler(sl::Mat& camera_frame_left, sl::Mat& camera_frame_right, std::shared_mutex &mutex);
    ~CameraHandler();                // Destructor
    
    /**
     * @brief Returns the state of the camera
     * 
     * @return 
     */
    sl::ERROR_CODE GetCameraState();
    //used in a thread

    /**
     * @brief Runs in a thread. Continous grabbing of the camera's frames
     * 
     */
    void StartCamera();

private:
    sl::Camera m_zed;
    bool m_failure;
    sl::ERROR_CODE m_returned_state;
    sl::Mat m_grabbed_frame_left;
    sl::Mat m_grabbed_frame_right;

    std::shared_mutex& m_mutex;
    sl::Mat& m_camera_frame_ref_left;
    sl::Mat& m_camera_frame_ref_right;


    /**
     * Initializes the camera's parameters
     * 
     * @param zed
     */
    void InitCamera(sl::Camera &zed);

};