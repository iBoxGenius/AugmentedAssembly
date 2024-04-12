#include "AugmentedInstructions.h"


AugmentedInstructions::AugmentedInstructions(std::vector<std::vector<cv::Point>>& corners, cv::Mat& image, std::mutex& mutex): m_corners(corners), m_image(image), m_mutex(mutex)
{

}

AugmentedInstructions::~AugmentedInstructions()
{
}

void AugmentedInstructions::StartInstructions()
{

	while(true)
	{
		DrawCorners();
	}

}

void AugmentedInstructions::DrawCorners()
{
	if(!m_image.empty())
	{
		for(size_t i = 0; i < m_corners.size(); i++)
		{
			if(!m_corners[i].empty())
			{
				/*
				cv::line(DEMONSTRATION_FRAME_LEFT, m_corners[i][0], m_corners[i][1], cv::Scalar(0, 255, 0), 2);
				cv::line(DEMONSTRATION_FRAME_LEFT, m_corners[i][1], m_corners[i][2], cv::Scalar(0, 255, 0), 2);
				cv::line(DEMONSTRATION_FRAME_LEFT, m_corners[i][2], m_corners[i][3], cv::Scalar(0, 255, 0), 2);
				cv::line(DEMONSTRATION_FRAME_LEFT, m_corners[i][3], m_corners[i][0], cv::Scalar(0, 255, 0), 2);

				cv::imshow("DEMONSTRATION_LEFT", DEMONSTRATION_FRAME_LEFT);


				cv::line(DEMONSTRATION_FRAME_RIGHT, m_corners[i][0], m_corners[i][1], cv::Scalar(0, 255, 0), 2);
				cv::line(DEMONSTRATION_FRAME_RIGHT, m_corners[i][1], m_corners[i][2], cv::Scalar(0, 255, 0), 2);
				cv::line(DEMONSTRATION_FRAME_RIGHT, m_corners[i][2], m_corners[i][3], cv::Scalar(0, 255, 0), 2);
				cv::line(DEMONSTRATION_FRAME_RIGHT, m_corners[i][3], m_corners[i][0], cv::Scalar(0, 255, 0), 2);

				cv::imshow("DEMONSTRATION_RIGHT", DEMONSTRATION_FRAME_RIGHT);
				*/
			}

		}
	}
}
