
#include "CameraHandler.h"

/*CameraHandler::CameraHandler(): m_failure(false), m_new_frame(false)
{
    // Set configuration parameters
    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_parameters.camera_fps = 30;

    // Open the camera
    m_returned_state = m_zed.open(init_parameters);
    if(m_returned_state != sl::ERROR_CODE::SUCCESS) {
        std::cout << "Error occured " << m_returned_state <<  std::endl;
        m_failure = true;
    }

    //m_detector_frame;
    m_new_frame = true;
}*/

CameraHandler::CameraHandler(sl::Mat& camera_frame_left, sl::Mat& camera_frame_right, std::shared_mutex& mutex): m_camera_frame_ref_left(camera_frame_left), m_camera_frame_ref_right(camera_frame_right) ,m_failure(false), m_mutex(mutex)
{
    std::cout << "ZEDSDK version: " << m_zed.getSDKVersion() << std::endl;
    InitCamera(m_zed);
    /*
    // Set configuration parameters
    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_parameters.camera_fps = 30;

    // Open the camera
    m_returned_state = m_zed.open(init_parameters);
    if(m_returned_state != sl::ERROR_CODE::SUCCESS) {
        m_failure = true;
    }
    */

}


CameraHandler::~CameraHandler()
{
    m_zed.close();
}

sl::ERROR_CODE CameraHandler::GetCameraState()
{
    return m_returned_state;
}



void CameraHandler::StartCamera()
{
    while(true)
    {
        if(!m_failure)
        {
            m_returned_state = m_zed.grab();
            if(m_returned_state == sl::ERROR_CODE::SUCCESS)
            {
                m_zed.retrieveImage(m_grabbed_frame_left, sl::VIEW::LEFT, sl::MEM::CPU);
                m_zed.retrieveImage(m_grabbed_frame_right, sl::VIEW::RIGHT, sl::MEM::CPU);
            }
            {
                std::unique_lock<std::shared_mutex> lock(m_mutex);
                m_grabbed_frame_left.copyTo(m_camera_frame_ref_left);
                m_grabbed_frame_right.copyTo(m_camera_frame_ref_right);
            }
        }
    }
}






void CameraHandler::SetMatForDetectorSL(sl::Mat& camera_frame)
{
    m_camera_frame_ref_left = camera_frame;
}


void CameraHandler::SetWantNewFrameFlag()
{

}



/*********************************  Unified Camera initializer    *******************************************************/
void CameraHandler::InitCamera(sl::Camera& zed)
{
    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_parameters.camera_fps = 60;
    init_parameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    init_parameters.coordinate_units = sl::UNIT::MILLIMETER;
    m_returned_state = zed.open(init_parameters);
    zed.setCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS, 5);
    if(m_returned_state != sl::ERROR_CODE::SUCCESS) 
    {
        m_failure = true;
    }
}