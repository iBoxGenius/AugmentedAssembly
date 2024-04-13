#include "AugmentedInstructions.h"

AugmentedInstructions* AugmentedInstructions::this_ptr = nullptr;

AugmentedInstructions::AugmentedInstructions(std::vector<std::vector<std::vector<cv::Point>>>& corners, cv::Mat& camera_image, std::mutex& mutex, AssemblyStates& assembly_state, std::vector<unsigned>& step_indices, unsigned& parts_cnt):
	m_corners(corners), m_image_camera(camera_image), m_mutex(mutex), m_assembly_state(assembly_state), m_step_indices(step_indices), m_parts_cnt(parts_cnt)
{
	SetThisPtr(this);
}


AugmentedInstructions::~AugmentedInstructions()
{
	for(auto& vid : m_videos)
	{
		if(vid.isOpened())
		{
			vid.release();
		}
	}
}


void AugmentedInstructions::StartInstructions()
{
	m_found_part_cnt = std::vector<unsigned>(m_parts_cnt, 0);	//per each object

	if(!(m_parts_cnt % 2) || (m_parts_cnt == 0))
	{
		std::cout << "Insuffiecient nubmer of objects provided" << std::endl;
		m_steps_cnt = 0;
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

	while(true)
	{
		start_time = std::chrono::high_resolution_clock::now();


		m_image_camera.copyTo(m_image_text_video);
		InsertAnimation(m_image_text_video);
		m_image_text_video.copyTo(m_image_show);
		DrawInstructions();

		switch(m_assembly_state)
		{
		case AssemblyStates::AssemblyStart:
			if(m_changed_state)
			{
				SetAssemblyIndices();
				m_changed_state = false;
			}

			break;
		case AssemblyStates::AssemblyStep:
			if(m_changed_state)
			{
				SetAssemblyIndices();
				m_changed_state = false;
			}

			break;
		case AssemblyStates::AssemblyFinal:
			if(m_changed_state)
			{
				SetAssemblyIndices();
				m_changed_state = false;
			}

			break;
		default:
			break;
		}

		if(!m_found_parts)
		{
			unsigned detected_part_cnt = 0;
			for(auto& part_idx : m_step_indices)
			{
				if(m_found_part_cnt[part_idx] >= 200)
				{
					detected_part_cnt++;
				}
				if(detected_part_cnt >= m_step_indices.size())
				{
					m_found_parts = true;
					m_timer_id = SetTimer(NULL, 1, 5000, (TIMERPROC)AugmentedInstructions::TimerProcStatic);
				}
			}
		}

		end_time = std::chrono::high_resolution_clock::now();
		dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
		//std::cout << "AssemblyInstructions duration: " << dur << " ms" << std::endl;
	}

}


void AugmentedInstructions::DrawInstructions()
{
	Lines_label lines_label = Lines_label::Top;

	if(!m_image_camera.empty())
	{
		//#pragma omp simd
		for(auto& part_idx : m_step_indices)
		{
			for(size_t j = 0; j < m_corners[part_idx].size(); j++)
			{
				if(!m_corners[part_idx][j].empty())
				{
					m_found_part_cnt[part_idx]++;

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

void AugmentedInstructions::InsertAnimation(cv::Mat &camera_frame)
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

	/*
	cv::Mat alpha(video_frame.size(), CV_8UC1, cv::Scalar(255*0.5)); // Initialize alpha channel with full opacity
	cv::Mat imageWithAlpha;
	std::vector<cv::Mat> channels;
	channels.push_back(video_frame);
	channels.push_back(alpha);
	cv::merge(channels, imageWithAlpha);
	cvtColor(imageWithAlpha, video_frame, cv::COLOR_BGR2BGRA);
	auto xd = video_frame.channels();
	auto ch = imageWithAlpha.channels();
	*/

	video_frame.copyTo(m_image_text_video(cv::Rect(m_image_text_video.cols - video_frame.cols, 0, video_frame.cols, video_frame.rows)));
}


void AugmentedInstructions::DrawLabel(const std::vector<cv::Point>& corners, Lines_label& lines, const unsigned index)
{
	/*
	* Change so the label has a direct relationship with the 'corners' instead of the BB.
	*/
	auto r = cv::boundingRect(corners);
	cv::rectangle(m_image_show, r, cv::Scalar(0, 255, 0), 2);
	std::string label = std::to_string(index);
	cv::Point textOrg(r.x, r.y - 20);
	int fontFace = cv::FONT_HERSHEY_SIMPLEX;
	double fontScale = 1.2;
	cv::Scalar color(0, 0, 255);
	int thickness = 4;
	cv::putText(m_image_show, "["  + label + "]", textOrg, fontFace, fontScale, color, thickness);
}


void AugmentedInstructions::SetAssemblyIndices()
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


void AugmentedInstructions::SetTimerNextState()
{

}


void AugmentedInstructions::SetStateStepCallback()
{
	/*
	this->m_assembly_state = AssemblyStates::AssemblyStep;
	this->m_changed_state = true;
	auto ret = KillTimer(NULL, this->m_timer_id);
	*/
}
