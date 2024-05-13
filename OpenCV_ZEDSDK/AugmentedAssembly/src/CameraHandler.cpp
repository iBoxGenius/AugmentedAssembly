
#include "CameraHandler.h"

CameraHandler::CameraHandler(sl::Mat& camera_frame_left, sl::Mat& camera_frame_right, std::shared_mutex& mutex): m_camera_frame_ref_left(camera_frame_left), m_camera_frame_ref_right(camera_frame_right) ,m_failure(false), m_mutex(mutex)
{
    std::cout << "ZEDSDK version: " << m_zed.getSDKVersion() << std::endl;
    InitCamera(m_zed);
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

/*********************************  Unified Camera initializer    *******************************************************/
void CameraHandler::InitCamera(sl::Camera& zed)
{
    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_parameters.camera_fps = 60;
    m_returned_state = zed.open(init_parameters);
    zed.setCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS, 5);
    if(m_returned_state != sl::ERROR_CODE::SUCCESS) 
    {
        m_failure = true;
    }
}