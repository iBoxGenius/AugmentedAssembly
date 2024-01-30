
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

CameraHandler::CameraHandler(sl::Mat& detector_frame, std::shared_mutex& mutex): m_detector_frame(detector_frame), m_failure(false), m_new_frame_rq(false), m_mutex(mutex)
{
    // Set configuration parameters
    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_parameters.camera_fps = 30;

    // Open the camera
    m_returned_state = m_zed.open(init_parameters);
    if(m_returned_state != sl::ERROR_CODE::SUCCESS) {
        //std::cout << "Error occured " << m_returned_state << std::endl;
        m_failure = true;
    }


    m_new_frame_rq = true;
}




CameraHandler::~CameraHandler()
{

}

sl::ERROR_CODE CameraHandler::GetCameraState()
{
    return m_returned_state;
}



void CameraHandler::Start()
{
    while(true)
    {
        if(!m_failure)
        {
            m_returned_state = m_zed.grab();
            if(m_returned_state == sl::ERROR_CODE::SUCCESS)
            {
                m_zed.retrieveImage(m_grabbed_frame, sl::VIEW::LEFT, sl::MEM::CPU);
            }
        }

        if(m_new_frame_rq)
        {
            std::unique_lock<std::shared_mutex> lock(m_mutex);
            m_grabbed_frame.copyTo(m_detector_frame);
            //m_new_frame = false;
        }
    }
}






void CameraHandler::SetMatForDetectorSL(sl::Mat& detector_frame)
{
    m_detector_frame = detector_frame;
}


void CameraHandler::SetWantNewFrameFlag()
{

}


