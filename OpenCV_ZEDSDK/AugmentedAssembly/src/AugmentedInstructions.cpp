#include "AugmentedInstructions.h"

AugmentedInstructions* AugmentedInstructions::this_ptr = nullptr;

AugmentedInstructions::AugmentedInstructions(std::vector<std::vector<std::vector<cv::Point>>>& corners, cv::Mat& camera_image, std::mutex& mutex, AssemblyStates& assembly_state, std::vector<unsigned>& step_indices, unsigned& parts_cnt):
	m_corners(corners), m_image_camera(camera_image), m_mutex(mutex), m_assembly_state(assembly_state), m_step_indices(step_indices), m_parts_cnt(parts_cnt), m_timer_duration_ms(5000), m_detected_max(25)
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
	cv::rectangle(m_image_show, r, cv::Scalar(0, 255, 0), 2);
	cv::Point text_org(r.x, r.y - 20);
	double font_scale = 1.2;
	cv::Scalar colour(0, 0, 255);
	int thickness = 4;
	cv::putText(m_image_show, "["  + std::to_string(index) + "]", text_org, cv::FONT_HERSHEY_SIMPLEX, font_scale, colour, thickness);
}


bool AugmentedInstructions::AreSidesClose()
{
	//if the sides are close and have a similar angle  <30°

	m_corners[m_step_indices[0]][unsigned(m_sides_to_match[0])];		//m_corners[which_part][which_side]		//side of the 1st component
	m_corners[m_step_indices[1]][unsigned(m_sides_to_match[1])];		//m_corners[which_part][which_side]		//side of the 2nd component

	return true;
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
			m_step_indices.push_back(3);

			m_sides_to_match.push_back(Sides::Bottom);
			m_sides_to_match.push_back(Sides::Bottom);
		}

		if(m_steps_current_step == 2)
		{
			m_step_indices.push_back(0);
			m_step_indices.push_back(3);
			m_step_indices.push_back(4);

			//m_sides_to_match.push_back(Sides::Top);
			//m_sides_to_match.push_back(Sides::Bottom);
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


		std::cout << "Next State: Assembly Step [1]" << std::endl;
		break;
	case AssemblyStates::AssemblyStep:
		SetAssemblyIndices();
		if(m_steps_current_step < m_steps_cnt)
		{
			SetForNextState(3000, 20);
			m_steps_current_step++;

			std::cout << "Next State: Assembly Step [2]" << std::endl;
		}
		else
		{
			this->m_assembly_state = AssemblyStates::AssemblyFinal;
			SetForNextState(3000, 20);
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
					if(m_found_part_cnt[m_step_indices.back()] > m_detected_max)		//if the composed part is detected m_detected_max times 
					{
						m_found_parts = true;
						m_timer_id = SetTimer(NULL, 1, m_timer_duration_ms, (TIMERPROC)AugmentedInstructions::TimerProcStatic);
						break;
					}
				}
			}
			else
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

