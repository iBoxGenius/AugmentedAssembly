
#include "AugmentedAssembly.h"

void AugmentedAssembly::GetNumberOfParts(std::filesystem::path path_to_parts)
{
	for(auto& p : std::filesystem::directory_iterator(path_to_parts))
	{
		std::cout << p.path() << std::endl;
		m_parts_cnt++;
	}
}

AugmentedAssembly::AugmentedAssembly(): m_zed(m_grabbed_frame_left_SL, m_grabbed_frame_right_SL, m_mutex),
										m_detector(m_method, m_detector_frame_MAT_new, m_mutex_detector, m_detector_new_frame_rq), m_detector_new_frame_rq(true),
										m_instructions(m_scene_corners, m_grabbed_frame_left_MAT, m_grabbed_frame_right_MAT, m_assembly_state, m_step_indices, m_parts_cnt, m_keypoints_size)
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
	unsigned planes_cnt = 10;
	for(size_t i = 0; i < m_scene_corners.size(); i++)
	{
		m_scene_corners[i] = std::vector<std::vector<cv::Point>>(planes_cnt);
		for(size_t j = 0; j < m_scene_corners[i].size(); j++)
		{
		}

	}

	
	//to evade reallocations
	m_keypoints_scene.reserve(KP_MAX);
	if(m_method == Method::BRISK)
	{
		cv::Mat brisk_Descriptors(KP_MAX, m_detector.GetDescriptorSize(), CV_8U);
		m_descriptor_scene = brisk_Descriptors;
	}
}


void AugmentedAssembly::Start()
{
	if(m_zed.GetCameraState() == sl::ERROR_CODE::SUCCESS)
	{
		thread_camera = std::thread(&CameraHandler::StartCamera, std::ref(m_zed));
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

		while(true)
		{
			start_time_loop = std::chrono::high_resolution_clock::now();
			m_keypoints_size = m_keypoints_scene.size();

			try
			{
				if((m_grabbed_frame_left_SL.getHeight() != 0) && (m_grabbed_frame_left_SL.getWidth() != 0))
				{
					{
						std::shared_lock<std::shared_mutex> lock(m_mutex);
						slMat2cvMat(m_grabbed_frame_left_SL).copyTo(m_grabbed_frame_left_MAT);
						slMat2cvMat(m_grabbed_frame_right_SL).copyTo(m_grabbed_frame_right_MAT);
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
							m_mutex_parts.get()->at(part_idx).lock();
							m_parts_new_rq.get()->at(part_idx) = false;
							m_mutex_parts.get()->at(part_idx).unlock();
							m_cv_parts.get()->at(part_idx).notify_one();
							
						}

					}
					/*************************	Matcher request	 **********************************/

					/************************************* Keypoints = demonstration ****************************************************/

					if(PRINT_AA == 1)
					{
						keypoints_image_show = m_grabbed_frame_left_MAT.clone();
						keypoints_show = m_keypoints_scene;
						cv::Mat img_matches;
						cv::drawKeypoints(keypoints_image_show, keypoints_show, keypoints_image_show);
						cv::imshow("Keypoints", keypoints_image_show);
						cv::waitKey(5);
						keypoints_show.clear();
					}
					/********************************************************************************************************************/

					m_mutex_detector.lock();
					m_detector_new_frame_rq.store(false, std::memory_order_release);		//detector will process the next image -> allowed to detect
					m_mutex_detector.unlock();

			}

				if(PRINT_AA == 1)
				{
					end_time_loop = std::chrono::high_resolution_clock::now();
					dur_loop = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_loop - start_time_loop).count();
					std::cout << "Loop, time elapsed: " << dur_loop << " ms" << std::endl;
				}

			}
			catch(const std::out_of_range& e)
			{
				std::cerr << "Vector Out of Range Exception: " << e.what() << std::endl;
			}
		}
	}
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


char* AugmentedAssembly::GetLeftFrame()
{
	return m_instructions.GetLeftFrame();
}

char* AugmentedAssembly::GetRightFrame()
{
	return m_instructions.GetRigthFrame();
}

extern "C"
{
	__declspec(dllexport) AugmentedAssembly* AugmentedAssembly_Create() { return new AugmentedAssembly();}
	__declspec(dllexport) void AugmentedAssembly_Start(AugmentedAssembly* ptr) { ptr->Start(); }
	__declspec(dllexport) char* AugmentedAssembly_LeftFrame(AugmentedAssembly* ptr) { return ptr->GetLeftFrame(); }
	__declspec(dllexport) char* AugmentedAssembly_RightFrame(AugmentedAssembly* ptr) { return ptr->GetRightFrame(); }
	__declspec(dllexport) void AugmentedAssembly_Delete(AugmentedAssembly* ptr) { delete ptr; }
}


