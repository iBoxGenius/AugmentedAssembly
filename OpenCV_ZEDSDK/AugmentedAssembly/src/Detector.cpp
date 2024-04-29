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

/*
Detector::Detector(Method method, cv::Mat& camera_frame): m_camera_frame_MAT_ref(nullptr), 
{

}
*/


Detector::~Detector()
{
}

void Detector::DetectCompute(cv::Mat mask, CV_OUT std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
	auto end_time_loop = std::chrono::high_resolution_clock::now();
	auto start_time_loop = std::chrono::high_resolution_clock::now();
	auto dur_loop = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_loop - start_time_loop).count();

	
	HANDLE hThread = GetCurrentThread();
	//SetThreadAffinityMask(hThread, 3);
	//SetThreadPriority(hThread, THREAD_PRIORITY_HIGHEST);
	CloseHandle(hThread);
	

	bool request_not_fullfiled = true;
	while(true)
	{
		try
		{
			if(!m_camera_frame_MAT_ref.empty())
			{
				m_mutex.lock();
				request_not_fullfiled = m_new_frame_rq.load(std::memory_order_acquire);
				m_mutex.unlock();
				if(!request_not_fullfiled)
				{
					//std::this_thread::sleep_for(std::chrono::milliseconds(100));
					start_time_loop = std::chrono::high_resolution_clock::now();
					cv::cvtColor(m_camera_frame_MAT_ref, m_camera_frame_MAT_ref, cv::COLOR_BGR2GRAY);

					if(mask.empty())
					{
						if(m_method == Method::SIFT)
						{
							//m_detector_sift->detectAndCompute(m_camera_frame_MAT_ref, cv::noArray(), keypoints, descriptors, false);
							m_detector_sift->detect(m_camera_frame_MAT_ref, keypoints, cv::noArray());
							cv::KeyPointsFilter::runByImageBorder(keypoints, m_camera_frame_MAT_ref.size(), KP_RETAIN_BODER_SIZE);
							cv::KeyPointsFilter::retainBest(keypoints, static_cast<int>(keypoints.size() * KP_RETAIN));
							m_detector_sift->compute(m_camera_frame_MAT_ref, keypoints, descriptors);
						}

						if(m_method == Method::ORB)
						{
							//m_detector_orb->detectAndCompute(m_camera_frame_MAT_ref, cv::noArray(), keypoints, descriptors, false);
							m_detector_orb->detect(m_camera_frame_MAT_ref, keypoints, cv::noArray());
							cv::KeyPointsFilter::runByImageBorder(keypoints, m_camera_frame_MAT_ref.size(), KP_RETAIN_BODER_SIZE);
							cv::KeyPointsFilter::retainBest(keypoints, static_cast<int>(keypoints.size() * KP_RETAIN));
							m_detector_orb->compute(m_camera_frame_MAT_ref, keypoints, descriptors);
						}

						if(m_method == Method::BRISK)
						{
							//m_detector_brisk->detectAndCompute(m_camera_frame_MAT_ref, cv::noArray(), keypoints, descriptors, false);
							m_detector_brisk->detect(m_camera_frame_MAT_ref, keypoints, cv::noArray());
							//std::cout << "Keypoints ALL:   " << keypoints.size() << std::endl;
							cv::KeyPointsFilter::runByImageBorder(keypoints, m_camera_frame_MAT_ref.size(), KP_RETAIN_BODER_SIZE);
							//std::cout << "Keypoints BORDER " << keypoints.size() << std::endl;

							//cv::KeyPointsFilter::retainBest(keypoints, keypoints.size() * KP_RETAIN);
							cv::KeyPointsFilter::retainBest(keypoints, ((keypoints.size() * KP_RETAIN) > KP_MAX) ? (KP_MAX) : (static_cast<int>(keypoints.size() * KP_RETAIN)));
							//std::cout << "Keypoints BEST:  " << keypoints.size() << std::endl;

							m_detector_brisk->compute(m_camera_frame_MAT_ref, keypoints, descriptors);
							//std::cout << "-------------------------------" << std::endl;

						}
					}
					else
					{
						m_detector_sift->detectAndCompute(m_camera_frame_MAT_ref, mask, keypoints, descriptors, false);
					}

					{
						std::lock_guard<std::mutex> lock(m_mutex);
						m_new_frame_rq.store(true, std::memory_order_release);	//notify AugmentedAssembly object about a new request

					}
					/*
					m_mutex.lock();
					m_new_frame_rq.store(true, std::memory_order_release);	//notify AugmentedAssembly object about a new request
					m_mutex.unlock();
					*/
					end_time_loop = std::chrono::high_resolution_clock::now();
					dur_loop = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_loop - start_time_loop).count();
					//std::cout << "Detector: "<< dur_loop << "ms" << "\t" << descriptors.size() << std::endl;
				}
			}


		}
		catch(const std::out_of_range& e)
		{
			std::cerr << "Vector Out of Range Exception: " << e.what() << std::endl;
		}
	}
}

void Detector::SetMatForCameraSL(cv::Mat& camera_frame)
{
	m_camera_frame_MAT_ref = camera_frame;
}

int Detector::GetDescriptorSize()
{
	if(m_method == Method::SIFT)	
	{
		return m_detector_sift->descriptorSize();
	}

	if(m_method == Method::ORB)								
	{
		return m_detector_orb->descriptorSize();
	}

	if(m_method == Method::BRISK)								
	{
		return m_detector_brisk->descriptorSize();
	}

	return 0;
}
