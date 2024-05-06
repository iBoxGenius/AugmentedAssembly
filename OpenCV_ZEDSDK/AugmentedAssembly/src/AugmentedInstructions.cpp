#include "AugmentedInstructions.h"

#include <omp.h>


AugmentedInstructions* AugmentedInstructions::this_ptr = nullptr;

AugmentedInstructions::AugmentedInstructions(std::vector<std::vector<std::vector<cv::Point>>>& corners, cv::Mat& camera_image_left, cv::Mat& camera_image_right,std::mutex& mutex, AssemblyStates& assembly_state, std::vector<unsigned>& step_indices, unsigned& parts_cnt, int& keypoints_size):
	m_corners(corners), m_image_camera_left(camera_image_left), m_image_camera_right(camera_image_right), m_mutex(mutex), m_assembly_state(assembly_state), m_step_indices(step_indices), m_parts_cnt(parts_cnt), m_timer_duration_ms(5000), m_detected_max(99999),//m_detected_max(25)
	m_object_step_properties(2, { {std::vector<std::vector<AugmentedInstructions::Sides>>(2, std::vector<AugmentedInstructions::Sides>(2))}, {std::vector<std::vector<unsigned>>(2, std::vector<unsigned>(2))} }),
	m_blink_sides(2), m_keypoints_size(keypoints_size)
{
	m_sides_to_match = std::vector<AugmentedInstructions::Sides>(2);	//2 sides to connect
	SetThisPtr(this);

	m_image_left_unity = std::make_unique<char[]>(1280 * 720 * 4);
	m_image_right_unity = std::make_unique<char[]>(1280 * 720 * 4);

	//std::cout << omp_get_max_threads() << std::endl;
	//omp_set_num_threads((omp_get_max_threads() - 4) / 2);
}


AugmentedInstructions::~AugmentedInstructions()
{

}

void AugmentedInstructions::SortPointsCounterClockwise(std::vector<cv::Point>& points)
{
	cv::Point centroid(0, 0);
	for(const cv::Point& pt : points)
	{
		centroid += pt;
	}
	centroid.x /= points.size();
	centroid.y /= points.size();

	std::sort(points.begin(), points.end(), [&centroid](const cv::Point& a, const cv::Point& b) {
		return atan2(a.y - centroid.y, a.x - centroid.x) < atan2(b.y - centroid.y, b.x - centroid.x);
	});
}

void AugmentedInstructions::StartInstructions()
{
	m_found_part_cnt = std::vector<unsigned>(m_parts_cnt, 0);	//per each object

	if(!(m_parts_cnt % 2) || (m_parts_cnt == 0))
	{
		std::cout << "Insuffiecient nubmer of objects provided" << std::endl;
		m_steps_cnt = 0;
		return;
	}
	else
	{
		m_steps_cnt = (m_parts_cnt / 2);

		//for - get idx of steps
		//m_step_indices.push_back(3);
	}

	{
		//sides  [which_step][which_component][which_plane]
		//planes [which_step][which_component][which_plane]

		/*************************** STEP 1 **************************************************/
		m_object_step_properties[0].sides_to_match[0][0] = AugmentedInstructions::Sides::Top;
		m_object_step_properties[0].sides_to_match[0][1] = AugmentedInstructions::Sides::Bottom;
		m_object_step_properties[0].sides_to_match[1][0] = AugmentedInstructions::Sides::Top;
		m_object_step_properties[0].sides_to_match[1][1] = AugmentedInstructions::Sides::Bottom;

		m_object_step_properties[0].planes_to_connect[0][0] = 0;
		m_object_step_properties[0].planes_to_connect[0][1] = 0;
		m_object_step_properties[0].planes_to_connect[1][0] = 2;
		m_object_step_properties[0].planes_to_connect[1][1] = 1;
		/*************************** STEP 1 **************************************************/


		/*************************** STEP 2 **************************************************/

		m_object_step_properties[1].sides_to_match[0][0] = AugmentedInstructions::Sides::Top;
		m_object_step_properties[1].sides_to_match[0][1] = AugmentedInstructions::Sides::Bottom;
		m_object_step_properties[1].sides_to_match[1][0] = AugmentedInstructions::Sides::Top;
		m_object_step_properties[1].sides_to_match[1][1] = AugmentedInstructions::Sides::Bottom;

		std::vector<AugmentedInstructions::Sides> a(2);
		m_object_step_properties[1].sides_to_match.push_back(a);
		m_object_step_properties[1].sides_to_match.push_back(a);

		m_object_step_properties[1].sides_to_match[2][0] = AugmentedInstructions::Sides::Top;
		m_object_step_properties[1].sides_to_match[2][1] = AugmentedInstructions::Sides::Bottom;
		m_object_step_properties[1].sides_to_match[3][0] = AugmentedInstructions::Sides::Top;
		m_object_step_properties[1].sides_to_match[3][1] = AugmentedInstructions::Sides::Bottom;

		m_object_step_properties[1].planes_to_connect[0][0] = 0;
		m_object_step_properties[1].planes_to_connect[0][1] = 2;
		m_object_step_properties[1].planes_to_connect[1][0] = 2;
		m_object_step_properties[1].planes_to_connect[1][1] = 0;
		
		std::vector<unsigned> b(2);
		m_object_step_properties[1].planes_to_connect.push_back(b);
		m_object_step_properties[1].planes_to_connect.push_back(b);

		m_object_step_properties[1].planes_to_connect[2][0] = 1;
		m_object_step_properties[1].planes_to_connect[2][1] = 3;
		m_object_step_properties[1].planes_to_connect[3][0] = 3;
		m_object_step_properties[1].planes_to_connect[3][1] = 1;

		/*************************** STEP 2 **************************************************/
	}

	for(size_t i = 0; i < m_steps_cnt + 2; i++)
	{
		m_written_instructions.push_back(std::string());
	}

	if(m_steps_cnt + 2 >= 4)
	{
		m_written_instructions[0] = std::string("Make sure all the objects are visible");
		m_written_instructions[1] = std::string("Connect the object [2] with the object [1]");
		m_written_instructions[2] = std::string("Connect the object [3] with the object [0]");
		m_written_instructions[3] = std::string("Assembly has finished sucessfully !");
	}

	LoadStepAnimations(std::filesystem::path(std::filesystem::current_path() / ("resources/steps_animations")).make_preferred());

	auto end_time = std::chrono::high_resolution_clock::now();
	auto start_time = std::chrono::high_resolution_clock::now();
	auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

	SetAssemblyIndices();

	m_timer_fps_id = SetTimer(NULL, 4, 1000, (TIMERPROC)AugmentedInstructions::TimerFpsStatic);
	while(true)
	{
		start_time = std::chrono::high_resolution_clock::now();
		m_image_camera_left.copyTo(m_image_text_video);
		InsertAnimation();
		m_image_text_video.copyTo(m_image_show);
		DrawInstructions();
		CheckForNewState();
		end_time = std::chrono::high_resolution_clock::now();
		dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
		//std::cout << "AssemblyInstructions duration: " << dur << " ms" << std::endl;
	}

}



void AugmentedInstructions::DrawInstructions()
{
	if(!m_image_camera_left.empty())
	{
		for(auto& part_idx : m_step_indices)
		{
			for(size_t j = 0; j < m_corners[part_idx].size(); j++)
			{
				if(!m_corners[part_idx][j].empty())
				{	
					bool is_empty = false;
					for(auto& pt : m_corners[part_idx][j])
					{
						if(pt.x == 0 && pt.y == 0)
						{
							is_empty = true;
						}
					}

					InsertText();

					if(!is_empty)
					{
						m_found_part_cnt[part_idx]++;
						cv::line(m_image_show, m_corners[part_idx][j][0], m_corners[part_idx][j][1], cv::Scalar(255, 0, 0), 2);
						cv::line(m_image_show, m_corners[part_idx][j][1], m_corners[part_idx][j][2], cv::Scalar(255, 255, 0), 2);
						cv::line(m_image_show, m_corners[part_idx][j][2], m_corners[part_idx][j][3], cv::Scalar(0, 255, 255), 2);
						cv::line(m_image_show, m_corners[part_idx][j][3], m_corners[part_idx][j][0], cv::Scalar(255, 255, 255), 2);

						DrawLabel(m_corners[part_idx][j], part_idx);
						DetermineCorrectPlanes();

						if(m_draw_slice)
						{
							cv::circle(m_image_show, m_center, 20, cv::Scalar(0, 0, 255), 4);
							//m_center = cv::Point(-100, -100);
							cv::line(m_image_show, m_pt1, m_pt2, cv::Scalar(0, 0, 255), 2);
							cv::line(m_image_show, m_pt3, m_pt4, cv::Scalar(0, 0, 255), 2);
						}

						break;
					}
					else
					{
						continue;
					}




					/*	Oclusion TEST
					cv::line(m_image_show, cv::Point(488, 503), cv::Point(736, 496), cv::Scalar(0, 255, 0), 2);
					cv::line(m_image_show, cv::Point(736, 496), cv::Point(740, 661), cv::Scalar(0, 255, 0), 2);
					cv::line(m_image_show, cv::Point(740, 661), cv::Point(492, 668), cv::Scalar(0, 255, 0), 2);
					cv::line(m_image_show, cv::Point(492, 668), cv::Point(488, 503), cv::Scalar(0, 255, 0), 2);
					*/



					

					/*
					cv::line(DEMONSTRATION_FRAME_RIGHT, m_corners[part_idx][j][0], m_corners[part_idx][j][1], cv::Scalar(0, 255, 0), 2);
					cv::line(DEMONSTRATION_FRAME_RIGHT, m_corners[part_idx][j][1], m_corners[part_idx][j][2], cv::Scalar(0, 255, 0), 2);
					cv::line(DEMONSTRATION_FRAME_RIGHT, m_corners[part_idx][j][2], m_corners[part_idx][j][3], cv::Scalar(0, 255, 0), 2);
					cv::line(DEMONSTRATION_FRAME_RIGHT, m_corners[part_idx][j][3], m_corners[part_idx][j][0], cv::Scalar(0, 255, 0), 2);
					*/
				}
			}

			if(!m_image_show.empty())
			{
				cv::imshow("DEMONSTRATION_LEFT", m_image_show);
				//cv::imshow("DEMONSTRATION_RIGHT", DEMONSTRATION_FRAME_RIGHT);
				//if(m_first_run)
				//{
					cv::waitKey(5);
				//	m_first_run = false;
				//}
				m_fps++;
			}
		}

	}

}


void AugmentedInstructions::LoadStepAnimations(std::filesystem::path path_to_steps)
{
	size_t i = 0;
	for(auto& p : std::filesystem::directory_iterator(path_to_steps))
	{
		if(p.path().extension() == ".mp4")
		{
			std::filesystem::path tmp = p.path();
			cv::String str(tmp.string());
			m_videos.push_back(cv::VideoCapture());
			m_videos[i].open(str, cv::CAP_FFMPEG);

			if(!m_videos[i].isOpened())
			{
				std::cout << "The video file could not be opened." << std::endl;
			}
			i++;
		}
	}
}

void AugmentedInstructions::InsertAnimation()
{
	if(m_assembly_state == AssemblyStates::AssemblyStep)
	{
		unsigned current_step = m_steps_current_step - 1;
		cv::Mat video_frame;
		m_videos[current_step] >> video_frame;
		if(video_frame.empty())
		{
			m_videos[current_step].set(cv::CAP_PROP_POS_FRAMES, 0);
			m_videos[current_step] >> video_frame;
		}

		cv::resize(video_frame, video_frame, cv::Size(220, 290), 0, 0, cv::INTER_AREA);
		cv::cvtColor(video_frame, video_frame, cv::COLOR_BGR2BGRA);
		video_frame.copyTo(m_image_text_video(cv::Rect(m_image_text_video.cols - video_frame.cols, 0, video_frame.cols, video_frame.rows)));
	}


	unsigned current_step = 0;
	cv::Point text_org(20, 50);
	double font_scale = 1.2;
	cv::Scalar colour(0, 255, 0);
	int thickness = 4;

	static int mem = 0;


	if(m_keypoints_size != 0)
	{
		mem = m_keypoints_size;
	}

	if(m_keypoints_size == 0)
	{
		m_keypoints_size = mem;
	}

	cv::putText(m_image_text_video, "KPs: " + std::to_string(m_keypoints_size), cv::Point(1065, 350), cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 0, 255), thickness);

}

void AugmentedInstructions::InsertText()
{
	unsigned current_step = 0;
	cv::Point text_org(20, 50);
	double font_scale = 1.2;
	cv::Scalar colour(0, 255, 0);
	int thickness = 4;
	switch(m_assembly_state)
	{
		case AssemblyStates::AssemblyStart:
			current_step = 0;
			//cv::putText(m_image_show, m_written_instructions[current_step], text_org, cv::FONT_HERSHEY_SIMPLEX, font_scale, colour, thickness);
			break;
		case AssemblyStates::AssemblyStep:
			current_step = m_steps_current_step;
			cv::putText(m_image_show, m_written_instructions[current_step], text_org, cv::FONT_HERSHEY_SIMPLEX, font_scale, colour, thickness);
			break;
		case AssemblyStates::AssemblyFinal:
			current_step = m_steps_current_step + 1;
			cv::putText(m_image_show, m_written_instructions[current_step], text_org, cv::FONT_HERSHEY_SIMPLEX, font_scale, colour, thickness);
			break;
		default:
			break;
	}


	//std::cout << m_keypoints_size << std::endl;


}


void AugmentedInstructions::DetermineCorrectPlanes()
{
	if(m_assembly_state == AssemblyStates::AssemblyStep)
	{
		unsigned current_step = m_steps_current_step - 1;
		//the second object is the default -> m_corners[m_step_indices[1]]

		//determine which plane is visible from planes to connect
		int visible_idx_default = -1;
		int visible_idx_desired = -1;
		int visible_idx_other = -1;
		unsigned which_cmp_default = 1;
		std::vector<unsigned>* connectable_planes = nullptr;
		
		//if the timer isn't running == no need to calculate
		if(m_blink_sides[1].timer_blink_id == 0)
		{
			//find which plane is visible of the default component
			for(size_t i = 0; i < m_corners[m_step_indices[1]].size(); i++)
			{
				for(auto& pt : m_corners[m_step_indices[1]][i])	//each point
				{
					if(pt != cv::Point(0, 0))
					{
						visible_idx_default = i;
						if(visible_idx_default < 4 && m_blink_sides[1].visible_plane < 4)
						{
							if(m_blink_sides[1].visible_plane != visible_idx_default)
							{
								m_blink_sides[0].current_cycle = 0;
								m_blink_sides[1].current_cycle = 0;
							}
						}
						m_blink_sides[1].visible_plane = i;
						break;
					}
				}
			}

			//find which CONNECTABLE plane is visible for the default component - to get the relatioship of the [default <--> other]
			for(auto& plane_connection : m_object_step_properties[current_step].planes_to_connect)
			{
				for(auto& pt : m_corners[m_step_indices[which_cmp_default]][plane_connection[which_cmp_default]])	//each point
				{
					if(pt != cv::Point(0, 0))
					{
						visible_idx_desired = plane_connection[1];
						//connectable_planes = &plane_connection;
						m_blink_sides[0].connectable_planes = &plane_connection;
						m_blink_sides[1].connectable_planes = &plane_connection;
					}
				}
			}

			if(visible_idx_default >= 0 && visible_idx_default < 4)
			{
				if(visible_idx_default == visible_idx_desired)
				{
					m_blink_sides[1].is_correct_plane = true;
				}
				else
				{
					m_blink_sides[1].is_correct_plane = false;
				}

				//if the timer has not been already set && the current cycles are not fulfilled
				if(m_blink_sides[1].current_cycle < m_blink_sides[1].max_cycles)
				{
					BlinkSide(1, m_corners[m_step_indices[1]][visible_idx_default]);
				}
			}
		}


		//if the timer isn't running == no need to calculate
		if(m_blink_sides[0].timer_blink_id == 0)
		{
			//find which plane is visible for the other component
			for(size_t i = 0; i < m_corners[m_step_indices[0]].size(); i++)
			{
				for(auto& pt : m_corners[m_step_indices[0]][i])	//each point
				{
					if(pt != cv::Point(0, 0))
					{
						visible_idx_other = i;
						if(visible_idx_other < 4 && m_blink_sides[1].visible_plane < 4)
						{
							if(m_blink_sides[0].visible_plane != visible_idx_other)
							{
								m_blink_sides[0].current_cycle = 0;
							}
						}
						m_blink_sides[0].visible_plane = i;
						break;
					}
				}
			}

			if(visible_idx_other >= 0 && visible_idx_other < 4)
			{
				//if the timer has not been already set && the current cycles are not fulfilled
				if(m_blink_sides[0].current_cycle < m_blink_sides[0].max_cycles)
				{
					if(m_blink_sides[1].connectable_planes)
					{
						//if the currently visible plane for the other component is the same as for the pair [default <--> other]
						if((visible_idx_other == m_blink_sides[1].connectable_planes->at(0)))
						{
							m_blink_sides[0].is_correct_plane = true;
						}
						else
						{
							m_blink_sides[0].is_correct_plane = false;
						}
					}
					BlinkSide(0, m_corners[m_step_indices[0]][visible_idx_other]);
				}
			}
		}

		
	}
}


void AugmentedInstructions::BlinkSide(unsigned which_component, std::vector<cv::Point> corners)
{
	m_blink_sides[which_component].pts = corners;

	if(which_component == 0)
	{
		m_blink_sides[which_component].timer_blink_id = SetTimer(NULL, 2, 330, (TIMERPROC)AugmentedInstructions::TimerBlink0Static);
	}

	if(which_component == 1)
	{
		m_blink_sides[which_component].timer_blink_id = SetTimer(NULL, 3, 330, (TIMERPROC)AugmentedInstructions::TimerBlink1Static);
	}
}


void AugmentedInstructions::BlinkCallback(unsigned which_timer)
{
	m_blink_sides[which_timer].current_cnt++;
	if(m_blink_sides[which_timer].current_cnt > 6)
	{
		unsigned ret = KillTimer(NULL, m_blink_sides[which_timer].timer_blink_id);
		if(ret > 0)
		{
			ret = KillTimer(NULL, m_blink_sides[which_timer].timer_blink_id);
		}

		m_blink_sides[which_timer].current_cnt = 0;
		m_blink_sides[which_timer].timer_blink_id = 0;
		m_blink_sides[which_timer].current_cycle++;
	}
	else
	{
		if(m_blink_sides[which_timer].is_correct_plane)
		{
			cv::fillConvexPoly(m_image_show, m_blink_sides[which_timer].pts.data(), m_blink_sides[which_timer].pts.size(), cv::Scalar(0, 230, 0));
		}
		else
		{
			cv::fillConvexPoly(m_image_show, m_blink_sides[which_timer].pts.data(), m_blink_sides[which_timer].pts.size(), cv::Scalar(0, 0, 230));
		}
	}
}


bool AugmentedInstructions::HasStateChanged()
{
	if(m_changed_state_for_main)
	{
		m_changed_state_for_main = false;
		return true;
	}
	return false;
}


void AugmentedInstructions::DrawLabel(const std::vector<cv::Point>& corners, const unsigned index)
{
	/*
	* Change so the label has a direct relationship with the 'corners' instead of the BB.
	*/
	auto r = cv::boundingRect(corners);
	//cv::rectangle(m_image_show, r, cv::Scalar(0, 255, 0), 2);
	cv::Point text_org(r.x, r.y - 20);
	double font_scale = 1.2;
	cv::Scalar colour(0, 0, 255);
	int thickness = 4;
	cv::putText(m_image_show, "["  + std::to_string(index) + "]", text_org, cv::FONT_HERSHEY_SIMPLEX, font_scale, colour, thickness);
}


void AugmentedInstructions::CalculateCenter(const unsigned which_step_component, cv::Point& center)
{
	//m_corners[which_part][which_plane][which_point]
	unsigned current_step = m_steps_current_step - 1;
	int which_plane = -1;
	int which_side = -1;

	//get which_plane. which_side indices
	//which_plane in the m_corners vectors
	//which_side in the m_object_step_properties.sides_to_match
	for(size_t i = 0; i < m_object_step_properties[current_step].planes_to_connect.size(); i++)
	{
		unsigned idx_plane = m_object_step_properties[current_step].planes_to_connect[i][which_step_component];
		if(!m_corners[m_step_indices[which_step_component]][idx_plane].empty())
		{
			if(m_corners[m_step_indices[which_step_component]][idx_plane][0] != cv::Point(0, 0) &&
				m_corners[m_step_indices[which_step_component]][idx_plane][1] != cv::Point(0, 0) &&
				m_corners[m_step_indices[which_step_component]][idx_plane][2] != cv::Point(0, 0) &&
				m_corners[m_step_indices[which_step_component]][idx_plane][3] != cv::Point(0, 0))
			{
				which_plane = idx_plane;
				which_side = i;
				break;
			}
		}
	}


	if(which_plane >= 0 && which_side >= 0)
	{
		switch(m_object_step_properties[current_step].sides_to_match[which_side][which_step_component])
		{
		case Sides::Top:

			center.x = (m_corners[m_step_indices[which_step_component]][which_plane][1].x + m_corners[m_step_indices[which_step_component]][which_plane][0].x) / 2;
			center.y = (m_corners[m_step_indices[which_step_component]][which_plane][1].y + m_corners[m_step_indices[which_step_component]][which_plane][0].y) / 2;
			break;

		case Sides::Right:

			break;
		case Sides::Bottom:
			center.x = (m_corners[m_step_indices[which_step_component]][which_plane][3].x + m_corners[m_step_indices[which_step_component]][which_plane][2].x) / 2;
			center.y = (m_corners[m_step_indices[which_step_component]][which_plane][3].y + m_corners[m_step_indices[which_step_component]][which_plane][2].y) / 2;
			break;

		case Sides::Left:

			break;
		default:
			break;
		}
	}
}


void AugmentedInstructions::CalculateAngle(const unsigned& which_step_component, double& angle)
{
	double delta_x = 0, delta_y = 0;
	double angle_rad = 0;


	unsigned current_step = m_steps_current_step - 1;
	int which_plane = -1;
	int which_side = -1;

	//get which_plane. which_side indices
	//which_plane in the m_corners vectors
	//which_side in the m_object_step_properties.sides_to_match
	for(size_t i = 0; i < m_object_step_properties[current_step].planes_to_connect.size(); i++)
	{
		unsigned idx_plane = m_object_step_properties[current_step].planes_to_connect[i][which_step_component];
		if(!m_corners[m_step_indices[which_step_component]][idx_plane].empty())
		{
			if(m_corners[m_step_indices[which_step_component]][idx_plane][0] != cv::Point(0, 0) &&
				m_corners[m_step_indices[which_step_component]][idx_plane][1] != cv::Point(0, 0) &&
				m_corners[m_step_indices[which_step_component]][idx_plane][2] != cv::Point(0, 0) &&
				m_corners[m_step_indices[which_step_component]][idx_plane][3] != cv::Point(0, 0))
			{
				which_plane = idx_plane;
				which_side = i;
				// = i;
				break;
			}
		}
	}


	if(which_plane >= 0 && which_side >= 0)
	{
		switch(m_object_step_properties[current_step].sides_to_match[which_side][which_step_component])
		{
		case Sides::Top:

			delta_x = abs(m_corners[m_step_indices[which_step_component]][which_plane][1].x - m_corners[m_step_indices[which_step_component]][which_plane][0].x);
			delta_y = abs(m_corners[m_step_indices[which_step_component]][which_plane][1].y - m_corners[m_step_indices[which_step_component]][which_plane][0].y);
			angle_rad = atan2(delta_x, delta_y);
			angle = angle_rad * 180.0 / CV_PI;

			break;
		case Sides::Right:

			break;
		case Sides::Bottom:


			delta_x = abs(m_corners[m_step_indices[which_step_component]][which_plane][3].x - m_corners[m_step_indices[which_step_component]][which_plane][2].x);
			delta_y = abs(m_corners[m_step_indices[which_step_component]][which_plane][3].y - m_corners[m_step_indices[which_step_component]][which_plane][2].y);
			angle_rad = atan2(delta_x, delta_y);
			angle = angle_rad * 180.0 / CV_PI;

			break;
		case Sides::Left:

			break;
		default:
			break;
		}
	}
}


bool AugmentedInstructions::IsInsideComponent(const cv::Point& center, const unsigned& which_step_component)
{

	unsigned current_step = m_steps_current_step - 1;
	int which_plane = -1;
	for(size_t i = 0; i < m_object_step_properties[current_step].planes_to_connect.size(); i++)
	{
		unsigned idx_plane = m_object_step_properties[current_step].planes_to_connect[i][which_step_component];
		if(!m_corners[m_step_indices[which_step_component]][idx_plane].empty())
		{
			if(m_corners[m_step_indices[which_step_component]][idx_plane][0] != cv::Point(0, 0) &&
				m_corners[m_step_indices[which_step_component]][idx_plane][1] != cv::Point(0, 0) &&
				m_corners[m_step_indices[which_step_component]][idx_plane][2] != cv::Point(0, 0) &&
				m_corners[m_step_indices[which_step_component]][idx_plane][3] != cv::Point(0, 0))
			{
				which_plane = idx_plane;
				break;
			}
		}
	}


	cv::Point rect[4];
	rect[0] = m_corners[m_step_indices[which_step_component]][which_plane][0];
	rect[1] = m_corners[m_step_indices[which_step_component]][which_plane][1];
	rect[2] = m_corners[m_step_indices[which_step_component]][which_plane][2];
	rect[3] = m_corners[m_step_indices[which_step_component]][which_plane][3];
	std::vector<cv::Point> contour(rect, rect + 4);

	return (cv::pointPolygonTest(contour, center, false) > 0);

}




bool AugmentedInstructions::IsInsideSlice(const cv::Point& center_to_test, const cv::Point& center_from, const unsigned& which_step_component)	//c2, 1 
{
	float percentage = 0.20;
	cv::Point pp_1, pp_2;
	float delta_2x = 0;
	float delta_2y = 0;


	
	unsigned current_step = m_steps_current_step - 1;
	int which_plane = -1;
	int which_side = -1;

	//get which_plane. which_side indices
	//which_plane in the m_corners vectors
	//which_side in the m_object_step_properties.sides_to_match
	for(size_t i = 0; i < m_object_step_properties[current_step].planes_to_connect.size(); i++)
	{
		unsigned idx_plane = m_object_step_properties[current_step].planes_to_connect[i][which_step_component];
		if(!m_corners[m_step_indices[which_step_component]][idx_plane].empty())
		{
			if(m_corners[m_step_indices[which_step_component]][idx_plane][0] != cv::Point(0, 0) &&
				m_corners[m_step_indices[which_step_component]][idx_plane][1] != cv::Point(0, 0) &&
				m_corners[m_step_indices[which_step_component]][idx_plane][2] != cv::Point(0, 0) &&
				m_corners[m_step_indices[which_step_component]][idx_plane][3] != cv::Point(0, 0))
			{
				which_plane = idx_plane;
				which_side = i;
				break;
			}
		}
	}


	if(which_plane >= 0 && which_side >= 0)
	{
		switch(m_object_step_properties[current_step].sides_to_match[which_side][which_step_component])
		{
		case Sides::Top:

			pp_1 = center_from + percentage * (m_corners[m_step_indices[which_step_component]][which_plane][1] - center_from);
			pp_2 = center_from + percentage * (m_corners[m_step_indices[which_step_component]][which_plane][0] - center_from);
			delta_2x = m_corners[m_step_indices[which_step_component]][which_plane][1].x - m_corners[m_step_indices[which_step_component]][which_plane][0].x;
			delta_2y = m_corners[m_step_indices[which_step_component]][which_plane][1].y - m_corners[m_step_indices[which_step_component]][which_plane][0].y;
			break;
		case Sides::Right:

			break;
		case Sides::Bottom:

			pp_1 = center_from + percentage * (m_corners[m_step_indices[which_step_component]][which_plane][3] - center_from);
			pp_2 = center_from + percentage * (m_corners[m_step_indices[which_step_component]][which_plane][2] - center_from);
			delta_2x = m_corners[m_step_indices[which_step_component]][which_plane][3].x - m_corners[m_step_indices[which_step_component]][which_plane][2].x;
			delta_2y = m_corners[m_step_indices[which_step_component]][which_plane][3].y - m_corners[m_step_indices[which_step_component]][which_plane][2].y;
			break;
		case Sides::Left:

			break;
		default:
			break;
		}

		m_pt1 = cv::Point(0, 0);
		m_pt2 = cv::Point(0, 0);
		m_pt3 = cv::Point(0, 0);
		m_pt4 = cv::Point(0, 0);

		double slope = delta_2y / delta_2x;
		double perpendicular_slope = -1.0 / slope;
		unsigned length = 110;
		double dx = length / sqrt(1 + perpendicular_slope * perpendicular_slope);
		double dy = perpendicular_slope * dx;
		m_pt1 = cv::Point(pp_1.x - dx, pp_1.y - dy);
		m_pt2 = cv::Point(pp_1.x + dx, pp_1.y + dy);

		m_pt3 = cv::Point(pp_2.x - dx, pp_2.y - dy);
		m_pt4 = cv::Point(pp_2.x + dx, pp_2.y + dy);

		std::vector<cv::Point> quad;
		quad.push_back(m_pt1);
		quad.push_back(m_pt2);
		quad.push_back(m_pt3);
		quad.push_back(m_pt4);
		SortPointsCounterClockwise(quad);
		return (cv::pointPolygonTest(quad, center_to_test, false) >= 0);
	}
	return false;
}


bool AugmentedInstructions::AreSidesClose()
{
	//if the sides are close and have a similar angle  <30°
	if(m_step_indices.empty())
	{
		return false;
	}

	if(m_assembly_state == AssemblyStates::AssemblyStep)
	{
		cv::Point c0, c1;
		double angle_1, angle_2;

		m_center = cv::Point(-100, -100);
		m_pt1 = cv::Point(-100, -100);
		m_pt2 = cv::Point(-100, -100);
		m_pt3 = cv::Point(-100, -100);
		m_pt4 = cv::Point(-100, -100);
		
		CalculateCenter(0, c0);	//component 0
		CalculateCenter(1, c1); //component 1

		CalculateAngle(0, angle_1);
		CalculateAngle(1, angle_2);

		if(c0 != cv::Point(0, 0))
		{
			m_center = c0;
		}
		
		IsInsideSlice(c0, c1, 1);
		if(c0.x > 0 && c0.y > 0 && c1.x > 0 && c1.y > 0)											//if there is a center
		{
			if(IsAngleOk(angle_1, angle_2))												//if the angles are correct
			{
				if(IsInsideComponent(c0, 1))					//if center of component 0 is inside the component 1
				{
					if(IsInsideSlice(c0, c1, 1))				//if it is inside the slice of the component 1
					{
						//std::cout << "C0 Angle: " << angle_1 << "°   C1 = Angle: " << angle_2 << std::endl;
						return true;
					}
				}
			}
		}
	}
	return false;
}


void AugmentedInstructions::SetAssemblyIndices()
{
	switch(m_assembly_state)
	{
	case AssemblyStates::AssemblyStart:
		m_step_indices.clear();
		m_sides_to_match.clear();
		
		m_step_indices.push_back(0);
		//m_step_indices.push_back(1);
		//m_step_indices.push_back(2);
		//m_step_indices.push_back(3);
		//m_step_indices.push_back(4);

		

		/*
		m_step_indices.push_back(1);
		m_step_indices.push_back(2);
		m_step_indices.push_back(3);
		*/

		break;
	case AssemblyStates::AssemblyStep:
		m_step_indices.clear();
		m_sides_to_match.clear();

		if(m_steps_current_step == 1)
		{
			m_step_indices.push_back(1);
			m_step_indices.push_back(2);
			//m_step_indices.push_back(3);

			m_sides_to_match.push_back(Sides::Top);
			m_sides_to_match.push_back(Sides::Bottom);
		}

		if(m_steps_current_step == 2)
		{
			m_step_indices.push_back(0);
			m_step_indices.push_back(3);
			//m_step_indices.push_back(4);

			m_sides_to_match.push_back(Sides::Top);
			m_sides_to_match.push_back(Sides::Bottom);
		}

		break;
	case AssemblyStates::AssemblyStepFinal:
		m_step_indices.clear();
		m_sides_to_match.clear();

		if(m_steps_current_step == 1)
		{
			//m_step_indices.push_back(1);
			//m_step_indices.push_back(2);
			m_step_indices.push_back(3);

			m_sides_to_match.push_back(Sides::Top);
			m_sides_to_match.push_back(Sides::Bottom);
		}

		if(m_steps_current_step == 2)
		{
			//m_step_indices.push_back(0);
			//m_step_indices.push_back(3);
			m_step_indices.push_back(4);

			m_sides_to_match.push_back(Sides::Top);
			m_sides_to_match.push_back(Sides::Bottom);
		}

		break;
	case AssemblyStates::AssemblyFinal:
		m_step_indices.clear();
		m_sides_to_match.clear();

		m_step_indices.push_back(4);

		break;
	default:
		break;
	}
}


void AugmentedInstructions::SetForNextState(unsigned timer_dur, unsigned detect_cnt)
{
	
	this->m_changed_state = true;
	this->m_changed_state_for_main = true;
	unsigned ret = KillTimer(NULL, this->m_timer_id);

	m_detected_max = detect_cnt;
	m_timer_duration_ms = timer_dur;
	m_found_parts = false;
	std::fill(m_found_part_cnt.begin(), m_found_part_cnt.end(), 0);
}


void AugmentedInstructions::SetNextStateCallback()
{
	switch(m_assembly_state)
	{
	case AssemblyStates::AssemblyStart:
		this->m_assembly_state = AssemblyStates::AssemblyStep;
		SetAssemblyIndices();
		SetForNextState(3000, 100);
		m_draw_slice = true;

		std::cout << "Next State: Assembly Step [1]" << std::endl;
		break;
	case AssemblyStates::AssemblyStep:
		//m_step_indices.clear();
		//m_sides_to_match.clear();
		
		//reset for the next step
		for(auto& blink : m_blink_sides)
		{
			blink.current_cycle = 0;
		}

		SetAssemblyIndices();
		if(m_steps_current_step < m_steps_cnt)
		{
			SetForNextState(50, 5);
			//m_steps_current_step++;
			this->m_assembly_state = AssemblyStates::AssemblyStepFinal;
			//m_step_indices.clear();
			//m_sides_to_match.clear();
			SetAssemblyIndices();

			m_draw_slice = false;
			std::cout << "Next State: Assembly StepFinal [1]" << std::endl;
		}
		else
		{
			SetForNextState(50, 5);
			this->m_assembly_state = AssemblyStates::AssemblyStepFinal;
			//m_step_indices.clear();
			//m_sides_to_match.clear();
			SetAssemblyIndices();

			m_draw_slice = false;
			std::cout << "Next State: Assembly StepFinal [2]" << std::endl;
		}


		break;

	case AssemblyStates::AssemblyStepFinal:
		//m_step_indices.clear();
		//m_sides_to_match.clear();
		SetAssemblyIndices();

		if(m_steps_current_step < m_steps_cnt)
		{
			SetForNextState(3000, 20);
			this->m_assembly_state = AssemblyStates::AssemblyStep;
			m_steps_current_step++;

			//m_step_indices.clear();
			//m_sides_to_match.clear();
			SetAssemblyIndices();
			m_draw_slice = true;
			std::cout << "Next State: Assembly Step[2]" << std::endl;
		}
		else
		{
			SetForNextState(3000, 20);
			this->m_assembly_state = AssemblyStates::AssemblyFinal;
			//m_step_indices.clear();
			//m_sides_to_match.clear();
			SetAssemblyIndices();

			m_draw_slice = false;
			std::cout << "Next State: Assembly Final" << std::endl;
		}

		break;
	case AssemblyStates::AssemblyFinal:
		SetAssemblyIndices();
		std::cout << "Assembly Final was reached" << std::endl;
		SetForNextState(3000, 99999999);

		break;


	default:
		this->m_assembly_state = AssemblyStates::AssemblyStart;
		SetAssemblyIndices();
		SetForNextState(3000, 100);

		break;
	}	
}


void AugmentedInstructions::FpsCallback()
{
	//std::cout << "FPS: " << m_fps << std::endl;
	m_fps = 0;
}


void AugmentedInstructions::CheckForNewState()
{
	if(!m_found_parts)
	{
		unsigned detected_part_cnt = 0;
		for(auto& part_idx : m_step_indices)
		{
			if(m_assembly_state == AssemblyStates::AssemblyStep)
			{
				if(AreSidesClose())
				{
					m_found_parts = true;
					m_timer_id = SetTimer(NULL, 1, m_timer_duration_ms, (TIMERPROC)AugmentedInstructions::TimerProcStatic);
					break;
				}
				return;
			}

			if(m_assembly_state == AssemblyStates::AssemblyStepFinal)
			{
				if(m_found_part_cnt[part_idx] >= m_detected_max)
				{
					detected_part_cnt++;
				}

				if(detected_part_cnt >= m_step_indices.size())
				{
					m_found_parts = true;
					m_timer_id = SetTimer(NULL, 1, m_timer_duration_ms, (TIMERPROC)AugmentedInstructions::TimerProcStatic);
				}
				//return;
			}

			if(m_assembly_state == AssemblyStates::AssemblyStart || m_assembly_state == AssemblyStates::AssemblyFinal)
			{
				if(m_found_part_cnt[part_idx] >= m_detected_max)
				{
					detected_part_cnt++;
				}

				if(detected_part_cnt >= m_step_indices.size())
				{
					m_found_parts = true;
					m_timer_id = SetTimer(NULL, 1, m_timer_duration_ms, (TIMERPROC)AugmentedInstructions::TimerProcStatic);
				}
			}
		}
	}
}


char* AugmentedInstructions::GetLeftFrame()
{
	if(m_image_camera_left.empty() || !m_image_camera_left.isContinuous())
	{
		return nullptr;
	}

	memcpy(m_image_left_unity.get(), m_image_camera_left.data, m_image_camera_left.total() * m_image_camera_left.elemSize());

	return m_image_left_unity.get();
}

char* AugmentedInstructions::GetRigthFrame()
{
	if(m_image_camera_right.empty() || !m_image_camera_right.isContinuous())
	{
		return nullptr;
	}

	memcpy(m_image_right_unity.get(), m_image_camera_right.data, m_image_camera_right.total() * m_image_camera_right.elemSize());

	return m_image_right_unity.get();
}


