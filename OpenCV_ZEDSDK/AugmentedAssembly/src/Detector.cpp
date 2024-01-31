#include "Detector.h"



Detector::Detector(cv::Mat& camera_frame, std::mutex& mutex, std::atomic<bool>& sync_var):  m_camera_frame_MAT_ref(camera_frame), m_mutex(mutex), m_new_frame_rq(sync_var)
{
	m_detector = cv::SIFT::create();
}

Detector::Detector(int nfeatures, int nOctaveLayers, double contrastThreshold, double edgeThreshold, double sigma, bool enable_precise_upscale, cv::Mat& camera_frame, std::mutex& mutex, std::atomic<bool>& sync_var)
	: m_camera_frame_MAT_ref(camera_frame), m_mutex(mutex), m_new_frame_rq(sync_var)
{
	m_detector = cv::SIFT::create(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma, enable_precise_upscale);
}

Detector::Detector(size_t max_keypoints, cv::Mat& camera_frame, std::mutex& mutex, std::atomic<bool>& sync_var): m_camera_frame_MAT_ref(camera_frame), m_mutex(mutex), m_new_frame_rq(sync_var)
{
	m_detector = cv::SIFT::create(max_keypoints);
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
					m_detector->detectAndCompute(m_camera_frame_MAT_ref, cv::noArray(), keypoints, descriptors, false);
				}
				else
				{
					m_detector->detectAndCompute(m_camera_frame_MAT_ref, mask, keypoints, descriptors, false);
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
