
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

	//m_parts_cnt = 5;
}


AugmentedAssembly::AugmentedAssembly(): m_zed(m_grabbed_frame_left_SL, m_grabbed_frame_right_SL, m_mutex), m_detector(m_method, m_detector_frame_MAT_new, m_mutex_detector, m_detector_new_frame_rq), m_detector_new_frame_rq(true)
{
	auto camera_ret_state = m_zed.GetCameraState();
	if(camera_ret_state  != sl::ERROR_CODE::SUCCESS)
	{
		std::cout << "Error occured " << camera_ret_state << std::endl;
	}

	GetNumberOfParts(std::filesystem::path(std::filesystem::current_path() / ("resources")).make_preferred());
	m_parts_new_rq = std::make_unique < std::vector<uint8_t>>(m_parts_cnt);
	m_mutex_parts = std::make_unique < std::vector<std::mutex>>(m_parts_cnt);
	m_cv_parts = std::make_unique < std::vector<std::condition_variable_any>>(m_parts_cnt);

	for(size_t i = 0; i < m_parts_cnt; i++)
	{
		m_parts_new_rq.get()->at(i) = true;
		m_assembly_parts.push_back(AssemblyPart(m_method, std::filesystem::path(std::filesystem::current_path() / ("resources/object" + std::to_string(i))).make_preferred(),
																				std::filesystem::path(std::filesystem::current_path() / ("resources/object" + std::to_string(i))).make_preferred(),
																				m_mutex_parts.get()->at(i), m_parts_new_rq.get()->at(i), m_cv_parts.get()->at(i)));
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
	/*m_window_handle = CreateWindowW(L"STATIC", L"Timer_AA", WS_OVERLAPPEDWINDOW,
									CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT,
									NULL, NULL, NULL, NULL);
									*/



	SetAssemblyIndices();
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

		std::vector<std::vector<std::vector<cv::Point>>> scene_corners(m_parts_cnt);

		/*
		* i => per object
		* j => per side
		* k => per corner
		*/
		for(size_t i = 0; i < scene_corners.size(); i++)
		{
			scene_corners[i] = std::vector<std::vector<cv::Point>>(6);
			for(size_t j = 0; j < scene_corners[i].size(); j++)
			{
				/*
				scene_corners[i][j].push_back(cv::Point(0, 0));
				scene_corners[i][j].push_back(cv::Point(0, 0));
				scene_corners[i][j].push_back(cv::Point(0, 0));
				scene_corners[i][j].push_back(cv::Point(0, 0));
				*/
			}

		}


		for(size_t i = 0; i < m_parts_cnt; i++)
		{
			m_threads_parts.push_back(std::move(std::thread(&AssemblyPart::FindMatches, std::ref(m_assembly_parts[i]), std::ref(m_descriptor_scene), std::ref(m_keypoints_scene), std::ref(scene_corners[i]))));
		}

		bool time_s = true;
		auto end_time = std::chrono::high_resolution_clock::now();
		auto start_time = std::chrono::high_resolution_clock::now();
		auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
		bool detector_requested = false;
		bool matcher_requested = false;


		auto end_time_loop = std::chrono::high_resolution_clock::now();
		auto start_time_loop = std::chrono::high_resolution_clock::now();
		auto dur_loop = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_loop - start_time_loop).count();

		cv::Mat keypoints_image_show;
		std::vector<cv::KeyPoint> keypoints_show;

		//std::cout << "Number of threads:  " << std::thread::hardware_concurrency() << std::endl;

		bool found = false;
		std::vector<unsigned> found_cnt(m_parts_cnt, 0);
		cv::Mat DEMONSTRATION_FRAME_LEFT;
		cv::Mat DEMONSTRATION_FRAME_RIGHT;


		while(true)
		{
			//start_time_loop = std::chrono::high_resolution_clock::now();
			try
			{
				switch(m_assembly_state)
				{
				case AssemblyStates::AssemblyStart:
					if(changed_state)
					{
						SetAssemblyIndices();
						changed_state = false;
					}

					break;
				case AssemblyStates::AssemblyStep:
					if(changed_state)
					{
						SetAssemblyIndices();
						changed_state = false;
					}

					break;
				case AssemblyStates::AssemblyFinal:
					if(changed_state)
					{
						SetAssemblyIndices();
						changed_state = false;
					}

					break;
				default:
					break;
				}



				/* Hladam assembly parts na zaciatku. Ak maju viac ako 200 najdeni, zacnem timer pre dalsi krok*/
				if(!found)
				{
					unsigned detected_part_cnt = 0;
					for(auto& part_idx : m_step_indices)
					{
						if(found_cnt[part_idx] >= 200)
						{
							detected_part_cnt++;
						}
						if(detected_part_cnt >= m_step_indices.size())
						{
							found = true;
							m_timer_id = SetTimer(NULL, 1, 5000, (TIMERPROC)TimerProcStatic);
						}
					}
				}
				


				if((m_grabbed_frame_left_SL.getHeight() != 0) && (m_grabbed_frame_left_SL.getWidth() != 0))
				{
					{
						std::shared_lock<std::shared_mutex> lock(m_mutex);
						slMat2cvMat(m_grabbed_frame_left_SL).copyTo(m_grabbed_frame_left_MAT);
						m_grabbed_frame_left_MAT.copyTo(DEMONSTRATION_FRAME_LEFT);
						slMat2cvMat(m_grabbed_frame_right_SL).copyTo(m_grabbed_frame_right_MAT);
						m_grabbed_frame_right_MAT.copyTo(DEMONSTRATION_FRAME_RIGHT);
					}
					if(!m_grabbed_frame_left_MAT.empty())
					{
						for(auto& part_idx : m_step_indices)
						{
							for(size_t j = 0; j < scene_corners[part_idx].size(); j++)
							{
								if(!scene_corners[part_idx][j].empty())
								{
									found_cnt[part_idx]++;

									cv::line(DEMONSTRATION_FRAME_LEFT, scene_corners[part_idx][j][0], scene_corners[part_idx][j][1], cv::Scalar(0, 255, 0), 2);
									cv::line(DEMONSTRATION_FRAME_LEFT, scene_corners[part_idx][j][1], scene_corners[part_idx][j][2], cv::Scalar(0, 255, 0), 2);
									cv::line(DEMONSTRATION_FRAME_LEFT, scene_corners[part_idx][j][2], scene_corners[part_idx][j][3], cv::Scalar(0, 255, 0), 2);
									cv::line(DEMONSTRATION_FRAME_LEFT, scene_corners[part_idx][j][3], scene_corners[part_idx][j][0], cv::Scalar(0, 255, 0), 2);

									cv::imshow("DEMONSTRATION_LEFT", DEMONSTRATION_FRAME_LEFT);


									cv::line(DEMONSTRATION_FRAME_RIGHT, scene_corners[part_idx][j][0], scene_corners[part_idx][j][1], cv::Scalar(0, 255, 0), 2);
									cv::line(DEMONSTRATION_FRAME_RIGHT, scene_corners[part_idx][j][1], scene_corners[part_idx][j][2], cv::Scalar(0, 255, 0), 2);
									cv::line(DEMONSTRATION_FRAME_RIGHT, scene_corners[part_idx][j][2], scene_corners[part_idx][j][3], cv::Scalar(0, 255, 0), 2);
									cv::line(DEMONSTRATION_FRAME_RIGHT, scene_corners[part_idx][j][3], scene_corners[part_idx][j][0], cv::Scalar(0, 255, 0), 2);

									cv::imshow("DEMONSTRATION_RIGHT", DEMONSTRATION_FRAME_RIGHT);
								}
							}
						}
						/*
						for(size_t i = 0; i < scene_corners.size(); i++)
						{
							for(size_t j = 0; j < scene_corners[i].size(); j++)
							{
								if(!scene_corners[i][j].empty())
								{
									found_cnt[i]++;

									cv::line(DEMONSTRATION_FRAME_LEFT, scene_corners[i][j][0], scene_corners[i][j][1], cv::Scalar(0, 255, 0), 2);
									cv::line(DEMONSTRATION_FRAME_LEFT, scene_corners[i][j][1], scene_corners[i][j][2], cv::Scalar(0, 255, 0), 2);
									cv::line(DEMONSTRATION_FRAME_LEFT, scene_corners[i][j][2], scene_corners[i][j][3], cv::Scalar(0, 255, 0), 2);
									cv::line(DEMONSTRATION_FRAME_LEFT, scene_corners[i][j][3], scene_corners[i][j][0], cv::Scalar(0, 255, 0), 2);

									cv::imshow("DEMONSTRATION_LEFT", DEMONSTRATION_FRAME_LEFT);


									cv::line(DEMONSTRATION_FRAME_RIGHT, scene_corners[i][j][0], scene_corners[i][j][1], cv::Scalar(0, 255, 0), 2);
									cv::line(DEMONSTRATION_FRAME_RIGHT, scene_corners[i][j][1], scene_corners[i][j][2], cv::Scalar(0, 255, 0), 2);
									cv::line(DEMONSTRATION_FRAME_RIGHT, scene_corners[i][j][2], scene_corners[i][j][3], cv::Scalar(0, 255, 0), 2);
									cv::line(DEMONSTRATION_FRAME_RIGHT, scene_corners[i][j][3], scene_corners[i][j][0], cv::Scalar(0, 255, 0), 2);

									cv::imshow("DEMONSTRATION_RIGHT", DEMONSTRATION_FRAME_RIGHT);

								}
							}

						}
						*/
					
						cv::waitKey(10);
						cv::imshow("Camera_left", m_grabbed_frame_left_MAT);
						cv::imshow("Camera_right", m_grabbed_frame_right_MAT);
					
					}
				}

				m_mutex_detector.lock();
				detector_requested = m_detector_new_frame_rq.load(std::memory_order_acquire);
				m_mutex_detector.unlock();

				if(detector_requested && !m_grabbed_frame_left_MAT.empty())									//detector finished 
				{
					/*
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
					std::cout << "	Keypoints size: " << m_keypoints_scene.size() << std::endl;
					std::cout << "-----------------------------" << std::endl;
					*/
					{
						std::shared_lock<std::shared_mutex> lock(m_mutex);
						m_grabbed_frame_left_MAT.copyTo(m_detector_frame_MAT_new);
						cv::cvtColor(m_detector_frame_MAT_new, m_detector_frame_MAT_new, cv::COLOR_BGR2GRAY);
					}
					//lock.unlock();

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


						//for(size_t i = 0; i < m_parts_cnt; i++)
						{
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
							

							if(matcher_requested)
							{
								//m_assembly_parts[i].SetNewSceneParam(m_descriptor_scene, m_keypoints_scene);
								//m_parts_new_rq.get()->at(i).store(false, std::memory_order_release);		//AssemblyPart(s) can try to match from a new set of keypoints/desc
							}
						}

						/*************************	Matcher request	 **********************************/


					keypoints_image_show = m_grabbed_frame_left_MAT.clone();
					keypoints_show = m_keypoints_scene;
					cv::Mat img_matches;
					/********************************************************************************************************************/

					m_mutex_detector.lock();
					m_detector_new_frame_rq.store(false, std::memory_order_release);		//detector will process the next image -> allowed to detect
					m_mutex_detector.unlock();
				

					/************************************* Keypoints = demonstration ****************************************************/
					cv::drawKeypoints(keypoints_image_show, keypoints_show, keypoints_image_show);
					cv::imshow("Keypoints", keypoints_image_show);
					keypoints_show.clear();
					/********************************************************************************************************************/

					/*drawMatches(m_assembly_parts[0].GetImages()[0], m_assembly_parts[0].GetKeypoints()[0], keypoints_image_show, m_keypoints_scene, m_assembly_parts[0].GetFilteredMatches()[0], img_matches, cv::Scalar::all(-1),
								cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT);
					if(!img_matches.empty())
					{
						cv::imshow("Good Matches", img_matches);
					}
					*/

				


			}
			//end_time_loop = std::chrono::high_resolution_clock::now();
			//dur_loop = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_loop - start_time_loop).count();

			//std::cout << "Loop, time elapsed:	" << dur_loop << " ms" << std::endl;
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
		thread_camera.join();
		thread_detector.join();
		//m_thread_instructions.join();

		for(auto& th : m_threads_parts)
		{
			th.join();
		}
	}
}


