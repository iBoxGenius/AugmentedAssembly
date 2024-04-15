#include "AugmentedInstructions.h"

AugmentedInstructions* AugmentedInstructions::this_ptr = nullptr;

AugmentedInstructions::AugmentedInstructions(std::vector<std::vector<std::vector<cv::Point>>>& corners, cv::Mat& camera_image, std::mutex& mutex, AssemblyStates& assembly_state, std::vector<unsigned>& step_indices, unsigned& parts_cnt):
	m_corners(corners), m_image_camera(camera_image), m_mutex(mutex), m_assembly_state(assembly_state), m_step_indices(step_indices), m_parts_cnt(parts_cnt), m_timer_duration_ms(5000), m_detected_max(20),//m_detected_max(25)
	m_object_step_properties(2, { {std::vector<std::vector<AugmentedInstructions::Sides>>(2, std::vector<AugmentedInstructions::Sides>(2))}, {std::vector<std::vector<unsigned>>(2, std::vector<unsigned>(2))} })
{
	m_sides_to_match = std::vector<AugmentedInstructions::Sides>(2);	//2 sides to connect
	SetThisPtr(this);

	
}


AugmentedInstructions::~AugmentedInstructions()
{

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
		m_object_step_properties[0].sides_to_match[0][0] = AugmentedInstructions::Sides::Top;
		m_object_step_properties[0].sides_to_match[0][1] = AugmentedInstructions::Sides::Bottom;
		m_object_step_properties[0].sides_to_match[1][0] = AugmentedInstructions::Sides::Top;
		m_object_step_properties[0].sides_to_match[1][1] = AugmentedInstructions::Sides::Bottom;

		m_object_step_properties[0].planes_to_connect[0][0] = 0;
		m_object_step_properties[0].planes_to_connect[0][1] = 0;
		m_object_step_properties[0].planes_to_connect[1][0] = 2;
		m_object_step_properties[0].planes_to_connect[1][1] = 1;


		m_object_step_properties[1].sides_to_match[0][0] = AugmentedInstructions::Sides::Top;
		m_object_step_properties[1].sides_to_match[0][1] = AugmentedInstructions::Sides::Bottom;
		m_object_step_properties[1].sides_to_match[1][0] = AugmentedInstructions::Sides::Top;
		m_object_step_properties[1].sides_to_match[1][1] = AugmentedInstructions::Sides::Bottom;

		m_object_step_properties[1].planes_to_connect[0][0] = 0;
		m_object_step_properties[1].planes_to_connect[0][1] = 2;
		m_object_step_properties[1].planes_to_connect[1][0] = 2;
		m_object_step_properties[1].planes_to_connect[1][1] = 0;
	}

	LoadStepAnimations(std::filesystem::path(std::filesystem::current_path() / ("resources/steps_animations")).make_preferred());

	auto end_time = std::chrono::high_resolution_clock::now();
	auto start_time = std::chrono::high_resolution_clock::now();
	auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();


	SetAssemblyIndices();
	while(true)
	{
		start_time = std::chrono::high_resolution_clock::now();

		m_image_camera.copyTo(m_image_text_video);
		InsertAnimation();
		m_image_text_video.copyTo(m_image_show);
		DrawInstructions();

		/*
		switch(m_assembly_state)
		{
		case AssemblyStates::AssemblyStart:
			if(m_changed_state)
			{
				m_changed_state_for_main = true;
				SetAssemblyIndices();
				m_changed_state = false;
			}
			CheckForNewState();

			break;
		case AssemblyStates::AssemblyStep:
			if(m_changed_state)
			{
				m_changed_state_for_main = true;
				SetAssemblyIndices();
				m_changed_state = false;
			}
			CheckForNewState();

			break;
		case AssemblyStates::AssemblyFinal:
			if(m_changed_state)
			{
				m_changed_state_for_main = true;
				SetAssemblyIndices();
				m_changed_state = false;
			}
			CheckForNewState();

			break;
		default:
			break;
		}
		*/

		CheckForNewState();
		switch(m_assembly_state)
		{
		case AssemblyStates::AssemblyStart:

			break;
		case AssemblyStates::AssemblyStep:
			
			break;
		case AssemblyStates::AssemblyFinal:

			break;
		default:
			break;
		}



		end_time = std::chrono::high_resolution_clock::now();
		dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
		//std::cout << "AssemblyInstructions duration: " << dur << " ms" << std::endl;
	}

}


void AugmentedInstructions::DrawInstructions()
{
	Sides lines_label = Sides::Top;

	if(!m_image_camera.empty())
	{
		//#pragma omp simd
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

					if(!is_empty)
					{
						m_found_part_cnt[part_idx]++;
					}

					cv::line(m_image_show, m_corners[part_idx][j][0], m_corners[part_idx][j][1], cv::Scalar(255, 0, 0), 2);
					cv::line(m_image_show, m_corners[part_idx][j][1], m_corners[part_idx][j][2], cv::Scalar(255, 255, 0), 2);
					cv::line(m_image_show, m_corners[part_idx][j][2], m_corners[part_idx][j][3], cv::Scalar(0, 255, 255), 2);
					cv::line(m_image_show, m_corners[part_idx][j][3], m_corners[part_idx][j][0], cv::Scalar(255, 255, 255), 2);

					DrawLabel(m_corners[part_idx][j], lines_label, part_idx);

					//cv::circle(m_image_show, m_center, 60, cv::Scalar(0, 0, 255), 4);
					if(m_draw_slice)
					{
						cv::circle(m_image_show, m_center, 20, cv::Scalar(0, 0, 255), 4);
						cv::line(m_image_show, m_pt1, m_pt2, cv::Scalar(0, 0, 255), 2);
						cv::line(m_image_show, m_pt3, m_pt4, cv::Scalar(0, 0, 255), 2);
					}

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
				cv::waitKey(10);
			}
		}

	}

}


void AugmentedInstructions::LoadStepAnimations(std::filesystem::path path_to_steps)
{
	for(auto& p : std::filesystem::directory_iterator(path_to_steps))
	{
		if(p.path().extension() == ".mp4")
			for(size_t i = 0; i < m_steps_cnt; i++)
			{
				std::filesystem::path tmp = p.path();
				cv::String str(tmp.string());
				m_videos.push_back(cv::VideoCapture());
				m_videos[i].open(str, cv::CAP_FFMPEG);

				if(!m_videos[i].isOpened())
				{
					std::cout << "The video file could not be opened." << std::endl;
				}
			}
	}
}

void AugmentedInstructions::InsertAnimation()
{
	cv::Mat video_frame;
	m_videos[0] >> video_frame;
	if(video_frame.empty())
	{
		m_videos[0].set(cv::CAP_PROP_POS_FRAMES, 0);
		m_videos[0] >> video_frame;
	}

	cv::resize(video_frame, video_frame, cv::Size(220, 290), 0, 0, cv::INTER_AREA);
	cv::cvtColor(video_frame, video_frame, cv::COLOR_BGR2BGRA);
	video_frame.copyTo(m_image_text_video(cv::Rect(m_image_text_video.cols - video_frame.cols, 0, video_frame.cols, video_frame.rows)));
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


void AugmentedInstructions::DrawLabel(const std::vector<cv::Point>& corners, Sides& lines, const unsigned index)
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
	
	switch(m_sides_to_match[which_step_component])
	{
	case Sides::Top:

		for(size_t i = 0; i < m_corners[m_step_indices[0]].size(); i++)
		{
			if(!m_corners[m_step_indices[which_step_component]][i].empty())
			{
				if(m_corners[m_step_indices[which_step_component]][i][0].x > 0 && m_corners[m_step_indices[which_step_component]][i][1].x > 0 &&
				   m_corners[m_step_indices[which_step_component]][i][0].y > 0 && m_corners[m_step_indices[which_step_component]][i][1].y > 0)
				{
					center.x = (m_corners[m_step_indices[which_step_component]][i][1].x + m_corners[m_step_indices[which_step_component]][i][0].x) / 2;
					center.y = (m_corners[m_step_indices[which_step_component]][i][1].y + m_corners[m_step_indices[which_step_component]][i][0].y) / 2;
					break;
				}
			}
		}
		break;
	case Sides::Right:

		break;
	case Sides::Bottom:

		for(size_t i = 0; i < m_corners[m_step_indices[0]].size(); i++)
		{
			if(!m_corners[m_step_indices[which_step_component]][i].empty())
			{
				if(m_corners[m_step_indices[which_step_component]][i][2].x > 0 && m_corners[m_step_indices[which_step_component]][i][3].x > 0 &&
				   m_corners[m_step_indices[which_step_component]][i][2].y > 0 && m_corners[m_step_indices[which_step_component]][i][3].y > 0)
				{
					center.x = (m_corners[m_step_indices[which_step_component]][i][3].x + m_corners[m_step_indices[which_step_component]][i][2].x) / 2;
					center.y = (m_corners[m_step_indices[which_step_component]][i][3].y + m_corners[m_step_indices[which_step_component]][i][2].y) / 2;
					break;
				}
			}
		}

		break;
	case Sides::Left:

		break;
	default:
		break;
	}
}


void AugmentedInstructions::CalculateAngle(const unsigned& which_step_component, double& angle)
{
	double delta_x = 0, delta_y = 0;
	double angle_rad = 0;
	switch(m_sides_to_match[which_step_component])
	{
	case Sides::Top:

		for(size_t i = 0; i < m_corners[m_step_indices[which_step_component]].size(); i++)
		{
			if(!m_corners[m_step_indices[which_step_component]][i].empty())
			{
				if(m_corners[m_step_indices[which_step_component]][i][0].x > 0 && m_corners[m_step_indices[which_step_component]][i][1].x > 0 &&
				   m_corners[m_step_indices[which_step_component]][i][0].y > 0 && m_corners[m_step_indices[which_step_component]][i][1].y > 0)
				{
					delta_x = abs(m_corners[m_step_indices[which_step_component]][i][1].x - m_corners[m_step_indices[which_step_component]][i][0].x);
					delta_y = abs(m_corners[m_step_indices[which_step_component]][i][1].y - m_corners[m_step_indices[which_step_component]][i][0].y);
					angle_rad = atan2(delta_x, delta_y);
					angle = angle_rad * 180.0 / CV_PI;
					break;
				}
			}
		}
		break;
	case Sides::Right:

		break;
	case Sides::Bottom:

		for(size_t i = 0; i < m_corners[m_step_indices[which_step_component]].size(); i++)
		{
			if(!m_corners[m_step_indices[which_step_component]][i].empty())
			{
				if(m_corners[m_step_indices[which_step_component]][i][2].x > 0 && m_corners[m_step_indices[which_step_component]][i][3].x > 0 &&
				   m_corners[m_step_indices[which_step_component]][i][2].y > 0 && m_corners[m_step_indices[which_step_component]][i][3].y > 0)
				{
					delta_x = abs(m_corners[m_step_indices[which_step_component]][i][3].x - m_corners[m_step_indices[which_step_component]][i][2].x);
					delta_y = abs(m_corners[m_step_indices[which_step_component]][i][3].y - m_corners[m_step_indices[which_step_component]][i][2].y);
					angle_rad = atan2(delta_x, delta_y);
					angle = angle_rad * 180.0 / CV_PI;
					break;
				}
			}
		}

		break;
	case Sides::Left:

		break;
	default:
		break;
	}
}


bool AugmentedInstructions::IsInsideComponent(const cv::Point& center, const unsigned& which_step_component)
{
	cv::Point rect[4];
	rect[0] = m_corners[m_step_indices[which_step_component]][0][0];
	rect[1] = m_corners[m_step_indices[which_step_component]][0][1];
	rect[2] = m_corners[m_step_indices[which_step_component]][0][2];
	rect[3] = m_corners[m_step_indices[which_step_component]][0][3];
	std::vector<cv::Point> contour(rect, rect + 4);

	return (cv::pointPolygonTest(contour, center, false) > 0);
}


bool AugmentedInstructions::IsInsideSlice(const cv::Point& center_to_test, const cv::Point& center_from, const unsigned& which_step_component)	//c2, 1 
{
	double percentage = 0.20;
	cv::Point pp_1, pp_2;
	double delta_2x = 0;
	double delta_2y = 0;

	switch(m_sides_to_match[which_step_component])
	{
	case Sides::Top:

		for(size_t i = 0; i < m_corners[m_step_indices[which_step_component]].size(); i++)
		{
			if(!m_corners[m_step_indices[which_step_component]][i].empty())
			{
				if(m_corners[m_step_indices[which_step_component]][i][0].x > 0 && m_corners[m_step_indices[which_step_component]][i][1].x > 0 &&
				   m_corners[m_step_indices[which_step_component]][i][0].y > 0 && m_corners[m_step_indices[which_step_component]][i][1].y > 0)
				{
					pp_1 = center_from + percentage * (m_corners[m_step_indices[which_step_component]][i][1] - center_from);
					pp_2 = center_from + percentage * (m_corners[m_step_indices[which_step_component]][i][0] - center_from);
					delta_2x = m_corners[m_step_indices[which_step_component]][i][1].x - m_corners[m_step_indices[which_step_component]][i][0].x;
					delta_2y = m_corners[m_step_indices[which_step_component]][i][1].y - m_corners[m_step_indices[which_step_component]][i][0].y;
					break;
				}
			}
		}
		break;
	case Sides::Right:

		break;
	case Sides::Bottom:

		for(size_t i = 0; i < m_corners[m_step_indices[which_step_component]].size(); i++)
		{
			if(!m_corners[m_step_indices[which_step_component]][i].empty())
			{
				if(m_corners[m_step_indices[which_step_component]][i][2].x > 0 && m_corners[m_step_indices[which_step_component]][i][3].x > 0 &&
				   m_corners[m_step_indices[which_step_component]][i][2].y > 0 && m_corners[m_step_indices[which_step_component]][i][3].y > 0)
				{
					pp_1 = center_from + percentage * (m_corners[m_step_indices[which_step_component]][i][3] - center_from);
					pp_2 = center_from + percentage * (m_corners[m_step_indices[which_step_component]][i][2] - center_from);
					delta_2x = m_corners[m_step_indices[which_step_component]][i][3].x - m_corners[m_step_indices[which_step_component]][i][2].x;
					delta_2y = m_corners[m_step_indices[which_step_component]][i][3].y - m_corners[m_step_indices[which_step_component]][i][2].y;
					break;
				}
			}
		}

		break;
	case Sides::Left:

		break;
	default:
		break;
	}


	double slope = delta_2y / delta_2x;
	double perpendicular_slope = -1.0 / slope;
	unsigned length = 110;
	double dx = length / sqrt(1 + perpendicular_slope * perpendicular_slope);
	double dy = perpendicular_slope * dx;
	m_pt1 = cv::Point(pp_1.x - dx, pp_1.y - dy);
	m_pt2 = cv::Point(pp_1.x + dx, pp_1.y + dy);

	m_pt3 = cv::Point(pp_2.x - dx, pp_2.y - dy);
	m_pt4 = cv::Point(pp_2.x + dx, pp_2.y + dy);

	cv::Point rect_slice[4];
	rect_slice[0] = m_pt1;
	rect_slice[1] = m_pt2;
	rect_slice[2] = m_pt3;
	rect_slice[3] = m_pt4;

	std::vector<cv::Point> contour_slice(rect_slice, rect_slice + 4);

	return (cv::pointPolygonTest(contour_slice, center_to_test, false) >= 0);
}


bool AugmentedInstructions::AreSidesClose()
{
	//if the sides are close and have a similar angle  <30°
	if(m_sides_to_match.empty() || m_step_indices.empty())
	{
		return false;
	}

	if(m_assembly_state == AssemblyStates::AssemblyStep)
	{
		cv::Point c0, c1;
		double angle_1, angle_2;

		CalculateCenter(0, c0);
		CalculateCenter(1, c1);

		CalculateAngle(0, angle_1);
		CalculateAngle(1, angle_2);

		//m_corners[which_part][which_plane][which_point]		//side of the 1st component
		//m_corners[which_part][which_plane][which_point]		//side of the 2nd component

		m_center = c0;
		IsInsideSlice(c0, c1, 1);

		if(c0.x > 0 && c0.y > 0 && c1.x > 0 && c1.y > 0)											//if there is a center
		{
			if(IsAngleOk(angle_1, angle_2))												//if the angles are correct
			{
				if(IsInsideComponent(c0, 1))					//if it is inside the other component
				{
					if(IsInsideSlice(c0, c1, 1))		//if it is inside the slice of the component
					{
						//std::cout << "C0 Angle: " << angle_1 << "°   C1 = Angle: " << angle_2 << std::endl;
						return true;
					}
				}
			}
		}
	}

	return false;
	
	//m_corners[which_part][which_plane][which_point]		//side of the 1st component
	//m_corners[which_part][which_plane][which_point]		//side of the 2nd component

	
	/*
	double c1x = 0, c1y = 0, c2x = 0, c2y = 0;
	double delta_1x = 0, delta_1y = 0, delta_2x = 0, delta_2y = 0;

	double angle_rad_1 = 0, angle_rad_2 = 0;
	double angle_deg_1 = 0, angle_deg_2 = 0;
	switch(m_sides_to_match[0])
	{
	case Sides::Top:
		c1x = (m_corners[m_step_indices[0]][0][0].x + m_corners[m_step_indices[0]][0][1].x) / 2;
		c1y = (m_corners[m_step_indices[0]][0][0].y + m_corners[m_step_indices[0]][0][1].y) / 2;

		delta_1x = abs(m_corners[m_step_indices[0]][0][1].x - m_corners[m_step_indices[0]][0][0].x);
		delta_1y = abs(m_corners[m_step_indices[0]][0][1].y - m_corners[m_step_indices[0]][0][0].y);
		angle_rad_1 = atan2(delta_1x, delta_1y);
		angle_deg_1 = angle_rad_1 * 180.0 / CV_PI;

		break;
	case Sides::Right:

		break;
	case Sides::Bottom:

		break;
	case Sides::Left:

		break;
	default:
		break;
	}

	switch(m_sides_to_match[1])
	{
	case Sides::Top:

		break;
	case Sides::Right:

		break;
	case Sides::Bottom:
		c2x = (m_corners[m_step_indices[1]][0][2].x + m_corners[m_step_indices[1]][0][3].x) / 2;
		c2y = (m_corners[m_step_indices[1]][0][2].y + m_corners[m_step_indices[1]][0][3].y) / 2;

		delta_2x = abs(m_corners[m_step_indices[1]][0][3].x - m_corners[m_step_indices[1]][0][2].x);
		delta_2y = abs(m_corners[m_step_indices[1]][0][3].y - m_corners[m_step_indices[1]][0][2].y);
		angle_rad_2 = atan2(delta_2x, delta_2y);
		angle_deg_2 = angle_rad_2 * 180.0 / CV_PI;

		break;
	case Sides::Left:

		break;
	default:
		break;
	}
	


	//			CALCULATE perpendicular line		//
	//cv::Point pp_1((2 * m_corners[m_step_indices[1]][0][3].x + m_corners[m_step_indices[1]][0][2].x) / 3, (2 * m_corners[m_step_indices[1]][0][3].y + m_corners[m_step_indices[1]][0][2].y) / 3);
	//cv::Point pp_2((m_corners[m_step_indices[1]][0][3].x + 2 * m_corners[m_step_indices[1]][0][2].x) / 3, (m_corners[m_step_indices[1]][0][3].y + 2 * m_corners[m_step_indices[1]][0][2].y) / 3);

	
	cv::Point rect[4];
	rect[0] = m_corners[m_step_indices[1]][0][0];
	rect[1] = m_corners[m_step_indices[1]][0][1];
	rect[2] = m_corners[m_step_indices[1]][0][2];
	rect[3] = m_corners[m_step_indices[1]][0][3];


	//			CALCULATE perpendicular line		//
	//cv::Point pp_1((2 * m_corners[m_step_indices[1]][0][3].x + m_corners[m_step_indices[1]][0][2].x) / 3, (2 * m_corners[m_step_indices[1]][0][3].y + m_corners[m_step_indices[1]][0][2].y) / 3);
	//cv::Point pp_2((m_corners[m_step_indices[1]][0][3].x + 2 * m_corners[m_step_indices[1]][0][2].x) / 3, (m_corners[m_step_indices[1]][0][3].y + 2 * m_corners[m_step_indices[1]][0][2].y) / 3);
	double percentage = 0.15;
	cv::Point pp_1 = cv::Point(c2x, c2y) + percentage * (m_corners[m_step_indices[1]][0][3] - cv::Point(c2x, c2y));
	cv::Point pp_2 = cv::Point(c2x, c2y) + percentage * (m_corners[m_step_indices[1]][0][2] - cv::Point(c2x, c2y));


	delta_2x = m_corners[m_step_indices[1]][0][3].x - m_corners[m_step_indices[1]][0][2].x;
	delta_2y = m_corners[m_step_indices[1]][0][3].y - m_corners[m_step_indices[1]][0][2].y;
	double slope = delta_2y / delta_2x;
	double perpendicular_slope = -1.0 / slope;
	unsigned length = 90;
	double dx = length / sqrt(1 + perpendicular_slope * perpendicular_slope);
	double dy = perpendicular_slope * dx;
	m_pt1 = cv::Point (pp_1.x - dx, pp_1.y - dy);
	m_pt2 = cv::Point (pp_1.x + dx, pp_1.y + dy);

	m_pt3 = cv::Point(pp_2.x - dx, pp_2.y - dy);
	m_pt4 = cv::Point(pp_2.x + dx, pp_2.y + dy);

	cv::Point rect_slice[4];
	rect_slice[0] = m_pt1;
	rect_slice[1] = m_pt2;
	rect_slice[2] = m_pt3;
	rect_slice[3] = m_pt4;

	m_center = cv::Point(c1x, c1y);
	std::vector<cv::Point> contour_slice(rect_slice, rect_slice + 4);
	std::vector<cv::Point> contour(rect, rect + 4);
	if(m_center.x > 0 && m_center.y > 0)							//if there is a center
	{
		if((abs(angle_deg_1 - angle_deg_2) < 25))			//if the angles are correct
		{
			if(cv::pointPolygonTest(contour, m_center, false) > 0)		//if it is inside the other component
			{
				if(cv::pointPolygonTest(contour_slice, m_center, false) > 0)	//if it is inside the slice of the component
				{
					//std::cout << "C0 Angle: " << angle_deg_1 << "°   C1 = Angle: " << angle_deg_2 << std::endl;
					return true;

				}
			}
		}
	}
	
	return false;
	*/
}


void AugmentedInstructions::SetAssemblyIndices()
{
	switch(m_assembly_state)
	{
	case AssemblyStates::AssemblyStart:
		m_step_indices.clear();
		m_sides_to_match.clear();
		
		m_step_indices.push_back(0);
		m_step_indices.push_back(1);
		m_step_indices.push_back(2);
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
		m_step_indices.clear();
		m_sides_to_match.clear();
		SetAssemblyIndices();
		if(m_steps_current_step < m_steps_cnt)
		{
			SetForNextState(50, 5);
			//m_steps_current_step++;
			this->m_assembly_state = AssemblyStates::AssemblyStepFinal;
			m_step_indices.clear();
			m_sides_to_match.clear();
			SetAssemblyIndices();

			m_draw_slice = false;
			std::cout << "Next State: Assembly StepFinal [1]" << std::endl;
		}
		else
		{
			SetForNextState(50, 5);
			this->m_assembly_state = AssemblyStates::AssemblyStepFinal;
			m_step_indices.clear();
			m_sides_to_match.clear();
			SetAssemblyIndices();

			m_draw_slice = false;
			std::cout << "Next State: Assembly StepFinal [2]" << std::endl;
		}


		break;

	case AssemblyStates::AssemblyStepFinal:
		m_step_indices.clear();
		m_sides_to_match.clear();
		SetAssemblyIndices();

		if(m_steps_current_step < m_steps_cnt)
		{
			SetForNextState(3000, 20);
			this->m_assembly_state = AssemblyStates::AssemblyStep;
			m_steps_current_step++;

			m_step_indices.clear();
			m_sides_to_match.clear();
			SetAssemblyIndices();
			m_draw_slice = true;
			std::cout << "Next State: Assembly Step[2]" << std::endl;
		}
		else
		{
			SetForNextState(3000, 20);
			this->m_assembly_state = AssemblyStates::AssemblyFinal;
			m_step_indices.clear();
			m_sides_to_match.clear();
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
					/*
					std::cout << "SidesAreClose" << std::endl;
					if(m_found_part_cnt[m_step_indices.back()] > m_detected_max)		//if the composed part is detected m_detected_max times 
					{
						m_found_parts = true;
						m_timer_id = SetTimer(NULL, 1, m_timer_duration_ms, (TIMERPROC)AugmentedInstructions::TimerProcStatic);
						break;
					}
					*/
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

