
#include "AugmentedAssembly.h"
#include <windows.h>


AugmentedAssembly* AugmentedAssembly::timerPtr = nullptr;


void AugmentedAssembly::GetNumberOfParts(std::filesystem::path path_to_parts)
{
	for(auto& p : std::filesystem::directory_iterator(path_to_parts))
	{
		std::cout << p.path() << std::endl;
		m_parts_cnt++;
	}

	//m_parts_cnt = 1;
}


AugmentedAssembly::AugmentedAssembly(): m_zed(m_grabbed_frame_left_SL, m_grabbed_frame_right_SL, m_mutex),
										m_detector(m_method, m_detector_frame_MAT_new, m_mutex_detector, m_detector_new_frame_rq), m_detector_new_frame_rq(true),
										m_instructions(m_scene_corners, m_grabbed_frame_left_MAT, m_mutex_instructions, m_assembly_state, m_step_indices, m_parts_cnt)
{										
	auto camera_ret_state = m_zed.GetCameraState();
	if(camera_ret_state  != sl::ERROR_CODE::SUCCESS)
	{
		std::cout << "Error occured " << camera_ret_state << std::endl;
	}

	GetNumberOfParts(std::filesystem::path(std::filesystem::current_path() / ("resources/objects")).make_preferred());
	m_parts_new_rq = std::make_unique < std::vector<uint8_t>>(m_parts_cnt);
	m_mutex_parts = std::make_unique < std::vector<std::mutex>>(m_parts_cnt);
	m_cv_parts = std::make_unique < std::vector<std::condition_variable_any>>(m_parts_cnt);
	m_scene_corners = std::vector<std::vector<std::vector<cv::Point>>>(m_parts_cnt);
	m_step_indices = std::vector<unsigned>(m_parts_cnt);

	for(size_t i = 0; i < m_parts_cnt; i++)
	{
		m_parts_new_rq.get()->at(i) = true;
		m_assembly_parts.push_back(AssemblyPart(m_method, std::filesystem::path(std::filesystem::current_path() / ("resources/objects/object" + std::to_string(i))).make_preferred(),
																				std::filesystem::path(std::filesystem::current_path() / ("resources/objects/object" + std::to_string(i))).make_preferred(),
																				m_mutex_parts.get()->at(i), m_parts_new_rq.get()->at(i), m_cv_parts.get()->at(i)));
	}


	
	/*
	* i => per object
	* j => per side
	* k => per corner
	*/
	for(size_t i = 0; i < m_scene_corners.size(); i++)
	{
		m_scene_corners[i] = std::vector<std::vector<cv::Point>>(6);
		for(size_t j = 0; j < m_scene_corners[i].size(); j++)
		{
			/*
			scene_corners[i][j].push_back(cv::Point(0, 0));
			scene_corners[i][j].push_back(cv::Point(0, 0));
			scene_corners[i][j].push_back(cv::Point(0, 0));
			scene_corners[i][j].push_back(cv::Point(0, 0));
			*/
		}

	}

	
	//to evade reallocations
	m_keypoints_scene.reserve(KP_MAX);
	if(m_method == Method::BRISK)
	{
		cv::Mat briskDescriptors(KP_MAX, m_detector.GetDescriptorSize(), CV_8U);
		m_descriptor_scene = briskDescriptors;
	}

	if(m_method == Method::ORB)
	{
		//cv::Mat Descriptors(KP_MAX, m_detector.GetDescriptorSize(), CV_8U);
		//m_descriptor_scene = Descriptors;
	}

	if(m_method == Method::SIFT)
	{
		//cv::Mat Descriptors(KP_MAX, m_detector.GetDescriptorSize(), CV_8U);
		//m_descriptor_scene = Descriptors;
	}
	



	/*
	m_step_indices.reserve(m_parts_cnt);

	if(!(m_parts_cnt % 2) || (m_parts_cnt == 0))
	{
		std::cout << "Insuffiecient nubmer of objects provided" << std::endl;
		m_steps_cnt = 0;
	}
	else
	{
		m_steps_cnt = (m_parts_cnt / 2) - 1;

		//for - get idx of steps
		//m_step_indices.push_back(3);
	}
	
	SetTimerPtr(this);
	m_window_handle = CreateWindowW(L"STATIC", L"Timer_AA", WS_OVERLAPPEDWINDOW,
									CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT,
									NULL, NULL, NULL, NULL);
									
	


	*/
}

/*
* 720p
loop duration = 11-16ms
detector loop = ~140-200ms (cluttered environment)
			  = ~100-130ms
* 1080p
loop duration = 11-30ms
detector loop = ~220-280ms (cluttered environment)
			  = ~190-230ms
	
Potential speed up with optimization flag -O2 and RELEASE version
*/

void AugmentedAssembly::Start()
{
	if(m_zed.GetCameraState() == sl::ERROR_CODE::SUCCESS)
	{
		thread_camera = std::thread(&CameraHandler::Start, std::ref(m_zed));
		thread_detector = std::thread(&Detector::DetectCompute, std::ref(m_detector), cv::Mat(), std::ref(m_keypoints_scene), std::ref(m_descriptor_scene));


		for(size_t i = 0; i < m_parts_cnt; i++)
		{
			m_threads_parts.push_back(std::move(std::thread(&AssemblyPart::FindMatches, std::ref(m_assembly_parts[i]), std::ref(m_descriptor_scene), std::ref(m_keypoints_scene), std::ref(m_scene_corners[i]))));
		}
		m_thread_instructions = std::thread(&AugmentedInstructions::StartInstructions, std::ref(m_instructions));

		bool detector_requested = false;
		bool matcher_requested = false;

		auto end_time_loop = std::chrono::high_resolution_clock::now();
		auto start_time_loop = std::chrono::high_resolution_clock::now();
		auto dur_loop = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_loop - start_time_loop).count();

		cv::Mat keypoints_image_show;
		std::vector<cv::KeyPoint> keypoints_show;

		//std::cout << "Number of threads:  " << std::thread::hardware_concurrency() << std::endl;

		while(true)
		{
			start_time_loop = std::chrono::high_resolution_clock::now();

			try
			{
				if((m_grabbed_frame_left_SL.getHeight() != 0) && (m_grabbed_frame_left_SL.getWidth() != 0))
				{
					{
						std::shared_lock<std::shared_mutex> lock(m_mutex);
						slMat2cvMat(m_grabbed_frame_left_SL).copyTo(m_grabbed_frame_left_MAT);
						slMat2cvMat(m_grabbed_frame_right_SL).copyTo(m_grabbed_frame_right_MAT);
					}

					if(!m_grabbed_frame_left_MAT.empty())
					{
						cv::waitKey(5);
						cv::imshow("Camera_left", m_grabbed_frame_left_MAT);
						cv::imshow("Camera_right", m_grabbed_frame_right_MAT);
					}
				}

				m_mutex_detector.lock();
				detector_requested = m_detector_new_frame_rq.load(std::memory_order_acquire);
				m_mutex_detector.unlock();

				if(detector_requested && !m_grabbed_frame_left_MAT.empty())									//detector finished 
				{
					{
						std::shared_lock<std::shared_mutex> lock(m_mutex);
						m_grabbed_frame_left_MAT.copyTo(m_detector_frame_MAT_new);
					}
					/************************************* Keypoints = demonstration ****************************************************/
				
					/*
					keypoints_image_show = m_grabbed_frame_left_MAT.clone();
					auto filtered = m_assembly_parts[0].GetFilteredMatches()[0];
					keypoints_show.clear();
					auto kp_scene = m_assembly_parts[0].GetKpSceneCopy();
					if(!kp_scene.empty())
					{
						for(size_t i = 0; i < filtered.size(); i++)
						{
							if(filtered[i].trainIdx <= kp_scene.size())
							{
								keypoints_show.push_back(kp_scene[filtered[i].trainIdx]);
							}
						}

						/*
						float minX = INT_MAX, minY = INT_MAX, maxX = INT_MIN, maxY = INT_MIN;
						for(const cv::KeyPoint& kp : keypoints_show) {
							minX = std::min(minX, kp.pt.x);
							minY = std::min(minY, kp.pt.y);
							maxX = std::max(maxX, kp.pt.x);
							maxY = std::max(maxY, kp.pt.y);
						}

						// Define the top-left and bottom-right corners of the rectangle
						cv::Point topLeft(minX, minY);
						cv::Point bottomRight(maxX, maxY);

						// Draw a rectangle around all the points
						cv::rectangle(keypoints_image_show, topLeft, bottomRight, cv::Scalar(0, 255, 0), 2);
						
					}
					*/

					/*************************	Matcher request	 **********************************/

					for(auto& part_idx : m_step_indices)
					{
						{
							std::unique_lock<std::mutex> lk(m_mutex_parts.get()->at(part_idx));
							matcher_requested = m_parts_new_rq.get()->at(part_idx);
						}

						if(matcher_requested)
						{
							m_assembly_parts[part_idx].SetNewSceneParam(m_descriptor_scene, m_keypoints_scene);

							if(m_instructions.HasStateChanged())
							{
								m_mutex_parts.get()->at(part_idx).lock();
								m_parts_new_rq.get()->at(part_idx) = false;
								m_mutex_parts.get()->at(part_idx).unlock();
							}
							else
							{
								m_parts_new_rq.get()->at(part_idx) = false;
							}
							m_cv_parts.get()->at(part_idx).notify_one();
						}

					}
					/*************************	Matcher request	 **********************************/

					/************************************* Keypoints = demonstration ****************************************************/

					keypoints_image_show = m_grabbed_frame_left_MAT.clone();
					keypoints_show = m_keypoints_scene;
					cv::Mat img_matches;
				

					//cv::drawKeypoints(keypoints_image_show, keypoints_show, keypoints_image_show);
					//cv::imshow("Keypoints", keypoints_image_show);
					keypoints_show.clear();
					/********************************************************************************************************************/

					/*
					drawMatches(m_assembly_parts[0].GetImages()[0], m_assembly_parts[0].GetKeypoints()[0], keypoints_image_show, m_keypoints_scene, m_assembly_parts[0].GetFilteredMatches()[0], img_matches, cv::Scalar::all(-1),
								cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT);
					if(!img_matches.empty())
					{
						cv::imshow("Good Matches", img_matches);
					}
					*/

					m_mutex_detector.lock();
					m_detector_new_frame_rq.store(false, std::memory_order_release);		//detector will process the next image -> allowed to detect
					m_mutex_detector.unlock();

			}
			end_time_loop = std::chrono::high_resolution_clock::now();
			dur_loop = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_loop - start_time_loop).count();

			//std::cout << "Loop, time elapsed: " << dur_loop << " ms" << std::endl;
			//std::cout << "-----------------------------" << std::endl;

			}
			catch(const std::out_of_range& e)
			{
				std::cerr << "Vector Out of Range Exception: " << e.what() << std::endl;
			}
		}
	}
}

void AugmentedAssembly::SetAssemblyIndices()
{
	static unsigned step_cnt = 0;
	switch(m_assembly_state)
	{
		case AssemblyStates::AssemblyStart:
			m_step_indices.clear();
			m_step_indices.push_back(0);
			m_step_indices.push_back(1);
			m_step_indices.push_back(2);

			break;
		case AssemblyStates::AssemblyStep:
			m_step_indices.clear();

			if(step_cnt == 1)
			{
				m_step_indices.push_back(1);
				m_step_indices.push_back(2);
				m_step_indices.push_back(3);
			}

			if(step_cnt == 2)
			{
				m_step_indices.push_back(3);
				m_step_indices.push_back(4);
			}

			break;
		case AssemblyStates::AssemblyFinal:
			m_step_indices.clear();
			m_step_indices.push_back(4);

			break;
		default:
			break;
	}
}


void AugmentedAssembly::TimerCallback()
{
	this->m_assembly_state = AssemblyStates::AssemblyFinal;
	this->changed_state = true;
	auto ret = KillTimer(NULL, this->m_timer_id);
	//this->m_timer_id = 0;
	//DestroyWindow(m_window_handle);
	//DWORD dw = GetLastError();
}


AugmentedAssembly::~AugmentedAssembly()
{
	if(m_zed.GetCameraState() == sl::ERROR_CODE::SUCCESS)
	{
		if(thread_camera.joinable())
		{
			thread_camera.join();
		}

		if(thread_detector.joinable())
		{
			thread_detector.join();
		}
		
		if(m_thread_instructions.joinable())
		{
			m_thread_instructions.join();
		}
		
		for(auto& th : m_threads_parts)
		{
			if(th.joinable())
			{
				th.join();
			}
		}
	}
}


