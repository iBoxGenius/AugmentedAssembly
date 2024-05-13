#include "Detector.h"



Detector::Detector(Method method, cv::Mat& camera_frame, std::mutex& mutex, std::atomic<bool>& sync_var):  m_camera_frame_MAT_ref(camera_frame), m_mutex(mutex), m_new_frame_rq(sync_var), m_method(method)
{

	if(m_method == Method::BRISK)								//90-130ms 120kp
	{
		m_detector_brisk = cv::BRISK::create();
	}
}


Detector::~Detector()
{
}


void Detector::DetectCompute(cv::Mat mask, CV_OUT std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
	auto end_time_loop = std::chrono::high_resolution_clock::now();
	auto start_time_loop = std::chrono::high_resolution_clock::now();
	auto dur_loop = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_loop - start_time_loop).count();
	
	HANDLE hThread = GetCurrentThread();
	SetThreadPriority(hThread, THREAD_PRIORITY_TIME_CRITICAL);
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
						if(m_method == Method::BRISK)
						{
							m_detector_brisk->detect(m_camera_frame_MAT_ref, keypoints, cv::noArray());
							cv::KeyPointsFilter::runByImageBorder(keypoints, m_camera_frame_MAT_ref.size(), KP_RETAIN_BODER_SIZE);
							cv::KeyPointsFilter::retainBest(keypoints, ((keypoints.size() * KP_RETAIN) > KP_MAX) ? (KP_MAX) : (static_cast<int>(keypoints.size() * KP_RETAIN)));
							m_detector_brisk->compute(m_camera_frame_MAT_ref, keypoints, descriptors);
						}
					}

					{
						std::lock_guard<std::mutex> lock(m_mutex);
						m_new_frame_rq.store(true, std::memory_order_release);	//notify AugmentedAssembly object about a new request
					}
					if(PRINT_DET == 1)
					{
						end_time_loop = std::chrono::high_resolution_clock::now();
						dur_loop = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_loop - start_time_loop).count();
						std::cout << "Detector: "<< dur_loop << "ms" << "\t" << descriptors.size() << std::endl;
					}
				}
			}


		}
		catch(const std::out_of_range& e)
		{
			std::cerr << "Vector Out of Range Exception: " << e.what() << std::endl;
		}
	}
}


int Detector::GetDescriptorSize()
{
	if(m_method == Method::BRISK)								
	{
		return m_detector_brisk->descriptorSize();
	}

	return 0;
}
