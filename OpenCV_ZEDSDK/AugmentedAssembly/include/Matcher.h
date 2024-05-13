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

    /**
     * @brief Constructor
     * 
     * \param method Method for the descriptor/keypoints
     * \param ratio_thresh Ratio for the ratio test
     */
    Matcher(Method method, const float ratio_thresh);
    ~Matcher();                // Destructor

    /**
     * @brief Matches the object's side descriptor to the scene
     * 
     * \param descriptor_object
     * \param descriptor_scene
     * \param keypoints_scene
     * \param good_matches_filtered [out]
     */
    void Match(const cv::Mat& descriptor_object, const cv::Mat& descriptor_scene,
               const std::vector<cv::KeyPoint>& keypoints_scene, std::vector<cv::DMatch>& good_matches_filtered);

private:
    cv::Ptr<cv::DescriptorMatcher> m_matcher_brisk;
    Method m_method;
    const float m_ratio_thresh;

    /**
     * @brief Filters by the Lowe's test (ratio test)
     * 
     * \param knn_matches [in]
     * \param good_matches [out]
     */
    void FilterOutliersLoweTest(std::vector< std::vector<cv::DMatch>>& knn_matches, std::vector<cv::DMatch>& good_matches);
};