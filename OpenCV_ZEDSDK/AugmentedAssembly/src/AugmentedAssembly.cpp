
#include "AugmentedAssembly.h"


AugmentedAssembly::AugmentedAssembly() : m_zed(m_grabbed_frame_SL, m_mutex), m_detector(m_detector_frame_MAT, m_mutex_detector, m_detector_new_frame_rq), m_detector_new_frame_rq(true)
{
	//m_zed.SetMatForDetectorSL(m_grabbed_frame_SL);
	//m_detector.SetMatForCameraSL(m_detector_frame_MAT);

	auto camera_ret_state = m_zed.GetCameraState();
	if(camera_ret_state  != sl::ERROR_CODE::SUCCESS)
	{
		std::cout << "Error occured " << camera_ret_state << std::endl;
	}
}


void AugmentedAssembly::Start()
{
	if(m_zed.GetCameraState() == sl::ERROR_CODE::SUCCESS)
	{
		thread_camera = std::thread(&CameraHandler::Start, std::ref(m_zed));
		thread_detector = std::thread(&Detector::DetectCompute, std::ref(m_detector), cv::Mat(), std::ref(m_keypoints), std::ref(m_descriptor));

		//m_mutex_detector.lock();
		//m_detector_new_frame_rq.store(true, std::memory_order_release);
		//m_mutex.unlock();

		bool time_s = true;
		auto end_time = std::chrono::high_resolution_clock::now();
		auto start_time = std::chrono::high_resolution_clock::now();
		auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
		bool detector_requested = false;
		while(true)
		{
			//cv::destroyAllWindows();

			if((m_grabbed_frame_SL.getHeight() != 0) || (m_grabbed_frame_SL.getWidth() != 0))
			{
				cv::waitKey(10);
				std::shared_lock<std::shared_mutex> lock(m_mutex);
				m_grabbed_frame_MAT = slMat2cvMat(m_grabbed_frame_SL);
				if(!m_grabbed_frame_MAT.empty())
				{
					cv::imshow("Camera", m_grabbed_frame_MAT);
				}
			}

			m_mutex_detector.lock();
			detector_requested = m_detector_new_frame_rq.load(std::memory_order_acquire);
			m_mutex_detector.unlock();
			if(detector_requested && !m_grabbed_frame_MAT.empty())
			{
				if(time_s)
				{
					start_time = std::chrono::high_resolution_clock::now();
					time_s = false;
				}
				else
				{
					end_time = std::chrono::high_resolution_clock::now();
					dur =  std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
					time_s = true;
				}
				std::cout << "Detector request, time elapsed:	" << dur << " ms" << std::endl;
				std::cout << "	Keypoints size: " << m_keypoints.size() << std::endl;
				std::cout << "	Descriptor size: " << m_descriptor.size() << std::endl;

				std::shared_lock<std::shared_mutex> lock(m_mutex);
				m_grabbed_frame_MAT.copyTo(m_detector_frame_MAT);
				
				m_mutex_detector.lock();
				m_detector_new_frame_rq.store(false, std::memory_order_release);		//detector can detect
				m_mutex_detector.unlock();
			}

		}
	}
}



AugmentedAssembly::~AugmentedAssembly()
{
	if(m_zed.GetCameraState() == sl::ERROR_CODE::SUCCESS)
	{
		thread_camera.join();
		thread_detector.join();
	}
}


