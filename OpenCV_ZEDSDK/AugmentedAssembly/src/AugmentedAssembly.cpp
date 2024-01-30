
#include "AugmentedAssembly.h"


AugmentedAssembly::AugmentedAssembly() : m_zed(m_detector_frame_SL, m_mutex), m_detector(m_detector_frame_MAT, m_mutex)
{
	m_zed.SetMatForDetectorSL(m_detector_frame_SL);
	m_detector.SetMatForCameraSL(m_detector_frame_MAT);

	auto camera_ret_state = m_zed.GetCameraState();
	if(camera_ret_state  != sl::ERROR_CODE::SUCCESS)
	{
		std::cout << "Error occured " << camera_ret_state << std::endl;
	}
}


void AugmentedAssembly::Start()
{
	thread_camera = std::thread(&CameraHandler::Start, std::ref(m_zed));
	thread_detector = std::thread(&Detector::DetectCompute, std::ref(m_detector), cv::Mat(), m_keypoints, m_descriptor, false);
	//thread_detector = std::thread([this]() { m_detector.DetectCompute(m_detector_frame_SL, cv::Mat(), m_keypoints, m_descriptor, false); });

	
	while(true)
	{
		//cv::destroyAllWindows();
		
		if((m_detector_frame_SL.getHeight() != 0) || (m_detector_frame_SL.getWidth() != 0))
		{
			cv::waitKey(10);
			std::shared_lock<std::shared_mutex> lock(m_mutex);
			m_detector_frame_MAT = slMat2cvMat(m_detector_frame_SL);
			if(!m_detector_frame_MAT.empty())
			{
				cv::imshow("Camera", m_detector_frame_MAT);
			}
		}

		
	}
	
}



AugmentedAssembly::~AugmentedAssembly()
{
	thread_camera.join();
	thread_detector.join();
}


