#pragma once

#include "Enums.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>


/*
* This class provides the AssemblyPart with good_matches per descriptor
*/
class Matcher {
public:
    Matcher(Method method, const float ratio_thresh);
    ~Matcher();                // Destructor

    void Match(const cv::Mat& descriptor_object, const cv::Mat& descriptor_scene,
               const std::vector<cv::KeyPoint>& keypoints_scene,std::vector<cv::DMatch>&good_matches);

private:
    cv::Ptr<cv::DescriptorMatcher> m_matcher_sift;
    cv::Ptr<cv::DescriptorMatcher> m_matcher_orb;
    cv::Ptr<cv::DescriptorMatcher> m_matcher_brisk;
    Method m_method;
    //std::vector<cv::Mat> m_descriptor;
    //std::vector<cv::DMatch> m_good_matches;
    const float m_ratio_thresh;

    void FilterOutliersLoweTest(std::vector< std::vector<cv::DMatch>>& knn_matches, std::vector<cv::DMatch>& good_matches);
    void DistanceFromCentroid(const std::vector<cv::Point2f>& points, cv::Point2f centroid, std::vector<double>& distances);
    void FilterOutliersSpatial(const std::vector<cv::KeyPoint>& keypoints_scene, std::vector<cv::DMatch>& good_matches);

};