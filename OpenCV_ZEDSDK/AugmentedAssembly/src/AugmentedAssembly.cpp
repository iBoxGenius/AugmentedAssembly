
#include "AugmentedAssembly.h"


AugmentedAssembly::AugmentedAssembly(): m_zed(m_grabbed_frame_SL, m_mutex), m_detector(m_method, m_detector_frame_MAT_new, m_mutex_detector, m_detector_new_frame_rq), m_detector_new_frame_rq(true), m_mutex_parts(m_parts_cnt)
{
	//m_zed.SetMatForDetectorSL(m_grabbed_frame_SL);
	//m_detector.SetMatForCameraSL(m_detector_frame_MAT);

	auto camera_ret_state = m_zed.GetCameraState();
	if(camera_ret_state  != sl::ERROR_CODE::SUCCESS)
	{
		std::cout << "Error occured " << camera_ret_state << std::endl;
	}


	for(size_t i = 0; i < m_parts_cnt; i++)
	{
		m_parts_new_rq[i] = true;
		m_assembly_parts.push_back(AssemblyPart(m_method, std::filesystem::path(std::filesystem::current_path() / ("resources/object" + std::to_string(i))).make_preferred(),
																				std::filesystem::path(std::filesystem::current_path() / ("resources/object" + std::to_string(i))).make_preferred(),
																				m_mutex_parts[i], m_parts_new_rq[i]));
	}
	

	//to evade reallocations
	m_keypoints_scene.reserve(KP_MAX);
	cv::Mat briskDescriptors(KP_MAX, m_detector.GetDescriptorSize(), CV_8U);		//only for BRISK
	m_descriptor_scene = briskDescriptors;
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

		std::vector<cv::Point2f> scene_corners(4);
		for(size_t i = 0; i < m_parts_cnt; i++)
		{
			m_threads_parts.push_back(std::move(std::thread(&AssemblyPart::FindMatches, std::ref(m_assembly_parts[i]), std::ref(m_descriptor_scene), std::ref(m_keypoints_scene), std::ref(scene_corners))));
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
		while(true)
		{
			//start_time_loop = std::chrono::high_resolution_clock::now();

			if((m_grabbed_frame_SL.getHeight() != 0) && (m_grabbed_frame_SL.getWidth() != 0))
			{
				std::shared_lock<std::shared_mutex> lock(m_mutex);
				slMat2cvMat(m_grabbed_frame_SL).copyTo(m_grabbed_frame_MAT);
				lock.unlock();
				if(!m_grabbed_frame_MAT.empty())
				{
					if(!scene_corners.empty())
					{
						auto images = m_assembly_parts[0].GetImages();

						/*
						cv::line(m_grabbed_frame_MAT, scene_corners[0], scene_corners[1], cv::Scalar(0, 255, 0), 2);
						cv::line(m_grabbed_frame_MAT, scene_corners[1], scene_corners[2], cv::Scalar(0, 255, 0), 2);
						cv::line(m_grabbed_frame_MAT, scene_corners[2], scene_corners[3], cv::Scalar(0, 255, 0), 2);
						cv::line(m_grabbed_frame_MAT, scene_corners[3], scene_corners[0], cv::Scalar(0, 255, 0), 2);
						*/
						line(m_grabbed_frame_MAT, scene_corners[0] + cv::Point2f((float)images[0].cols, 0),
							 scene_corners[1] + cv::Point2f((float)images[0].cols, 0), cv::Scalar(0, 255, 0), 4);
						line(m_grabbed_frame_MAT, scene_corners[1] + cv::Point2f((float)images[0].cols, 0),
							 scene_corners[2] + cv::Point2f((float)images[0].cols, 0), cv::Scalar(0, 255, 0), 4);
						line(m_grabbed_frame_MAT, scene_corners[2] + cv::Point2f((float)images[0].cols, 0),
							 scene_corners[3] + cv::Point2f((float)images[0].cols, 0), cv::Scalar(0, 255, 0), 4);
						line(m_grabbed_frame_MAT, scene_corners[3] + cv::Point2f((float)images[0].cols, 0),
							 scene_corners[0] + cv::Point2f((float)images[0].cols, 0), cv::Scalar(0, 255, 0), 4);
						
					}
					cv::waitKey(10);
					cv::imshow("Camera", m_grabbed_frame_MAT);
				}
			}

			m_mutex_detector.lock();
			detector_requested = m_detector_new_frame_rq.load(std::memory_order_acquire);
			m_mutex_detector.unlock();

			if(detector_requested && !m_grabbed_frame_MAT.empty())									//detector finished 
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
				std::shared_lock<std::shared_mutex> lock(m_mutex);
				m_grabbed_frame_MAT.copyTo(m_detector_frame_MAT_new);
				//lock.unlock();

				/************************************* Keypoints = demonstration ****************************************************/
				try
				{
					/*
					keypoints_image_show = m_grabbed_frame_MAT.clone();
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

					for(size_t i = 0; i < m_parts_cnt; i++)
					{
						m_mutex_parts[i].lock();
						matcher_requested = m_parts_new_rq[i].load(std::memory_order_acquire);
						m_mutex_parts[i].unlock();

						if(matcher_requested)
						{
							m_parts_new_rq[i].store(false, std::memory_order_release);		//AssemblyPart(s) can try to match from a new set of keypoints/desc
							m_assembly_parts[i].SetNewSceneParam(m_descriptor_scene, m_keypoints_scene);
						}
					}

					/*************************	Matcher request	 **********************************/
				}
				catch(const std::out_of_range& e)
				{
					std::cout << "Out of Range error.";

				}


				keypoints_image_show = m_grabbed_frame_MAT.clone();
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
	}
}



AugmentedAssembly::~AugmentedAssembly()
{
	if(m_zed.GetCameraState() == sl::ERROR_CODE::SUCCESS)
	{
		thread_camera.join();
		thread_detector.join();

		for(auto& th : m_threads_parts)
		{
			th.join();
		}
	}
}


