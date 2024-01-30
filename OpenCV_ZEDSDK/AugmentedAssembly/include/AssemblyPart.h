#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

class AssemblyPart {
public:
    AssemblyPart();                 // Default constructor
    ~AssemblyPart();                // Destructor

    void SetDescriptor(cv::Mat& desc);
    cv::Mat GetDescriptor();

private:
    std::vector<cv::Mat> m_images;
    std::vector<std::vector<cv::KeyPoint>> m_keypoints;       //keypoints per image
    cv::Mat m_descriptor;
    
};