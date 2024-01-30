#include "Detector.h"



Detector::Detector(cv::Mat& camera_frame, std::shared_mutex& mutex): m_new_frame_rq(false), m_camera_frame_MAT(camera_frame), m_mutex(mutex)
{
	cv::Ptr<cv::SIFT> detector = cv::SIFT::create();
}

Detector::Detector(int nfeatures, int nOctaveLayers, double contrastThreshold, double edgeThreshold, double sigma, bool enable_precise_upscale, cv::Mat& camera_frame, std::shared_mutex& mutex): m_new_frame_rq(false), m_camera_frame_MAT(camera_frame), m_mutex(mutex)
{
	cv::Ptr<cv::SIFT> detector = cv::SIFT::create(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma, enable_precise_upscale);
}

Detector::Detector(size_t max_keypoints, cv::Mat& camera_frame, std::shared_mutex& mutex): m_new_frame_rq(false), m_camera_frame_MAT(camera_frame), m_mutex(mutex)
{
	cv::Ptr<cv::SIFT> detector = cv::SIFT::create(max_keypoints);
}

Detector::~Detector()
{
}

void Detector::DetectCompute(cv::Mat mask, CV_OUT std::vector<cv::KeyPoint>& keypoints, cv::Mat descriptors, bool useProvidedKeypoints)
{
	while(true)
	{
		if(false) //m_new_frame
		{
			std::shared_lock<std::shared_mutex> lock(m_mutex);
			//m_camera_frame_MAT = slMat2cvMat(image);
			if(mask.empty())
			{
				m_detector->detectAndCompute(m_camera_frame_MAT, cv::noArray(), keypoints, descriptors);
			}
			else
			{
				m_detector->detectAndCompute(m_camera_frame_MAT, mask, keypoints, descriptors);
			}
			
			//m_new_frame_rq = false;
		}
	}
}

void Detector::SetMatForCameraSL(cv::Mat& camera_frame)
{
	m_camera_frame_MAT = camera_frame;
}

void Detector::SetNewFrameReadyFlag()
{
	
}

