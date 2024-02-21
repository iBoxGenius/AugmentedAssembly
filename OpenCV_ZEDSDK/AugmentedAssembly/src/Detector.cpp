#include "Detector.h"



Detector::Detector(Method method, cv::Mat& camera_frame, std::mutex& mutex, std::atomic<bool>& sync_var):  m_camera_frame_MAT_ref(camera_frame), m_mutex(mutex), m_new_frame_rq(sync_var), m_method(method)
{
	if(m_method == Method::SIFT)								//700-900ms 400kp		a random image
	{
		m_detector_sift = cv::SIFT::create();
	}

	if(m_method == Method::ORB)								//400-500ms  500kp
	{
		m_detector_orb = cv::ORB::create();
	}

	if(m_method == Method::BRISK)								//90-130ms 120kp
	{
		m_detector_brisk = cv::BRISK::create();
	}

	/*
	if(type == "SIFT_GPU")
	{
		//TO DO
	}
	*/
}

Detector::~Detector()
{
}

void Detector::DetectCompute(cv::Mat mask, CV_OUT std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
	bool request_not_fullfiled = true;
	while(true)
	{
		if(!m_camera_frame_MAT_ref.empty())
		{
			m_mutex.lock();
			request_not_fullfiled = m_new_frame_rq.load(std::memory_order_acquire);
			m_mutex.unlock();
			if(!request_not_fullfiled)
			{
				//std::this_thread::sleep_for(std::chrono::milliseconds(100));
				if(mask.empty())
				{
					if(m_method == Method::SIFT)
					{
						m_detector_sift->detectAndCompute(m_camera_frame_MAT_ref, cv::noArray(), keypoints, descriptors, false);
					}

					if(m_method == Method::ORB)
					{
						m_detector_orb->detectAndCompute(m_camera_frame_MAT_ref, cv::noArray(), keypoints, descriptors, false);
					}

					if(m_method == Method::BRISK)
					{
						m_detector_brisk->detectAndCompute(m_camera_frame_MAT_ref, cv::noArray(), keypoints, descriptors, false);
					}
				}
				else
				{
					m_detector_sift->detectAndCompute(m_camera_frame_MAT_ref, mask, keypoints, descriptors, false);
				}
				m_mutex.lock();
				m_new_frame_rq.store(true, std::memory_order_release);	//notify AugmentedAssembly object about a new request
				m_mutex.unlock();
			}
		}
	}
}

void Detector::SetMatForCameraSL(cv::Mat& camera_frame)
{
	m_camera_frame_MAT_ref = camera_frame;
}
