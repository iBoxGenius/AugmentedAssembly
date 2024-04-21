#include "Matcher.h"
#include <iostream>

Matcher::Matcher(Method method, const float ratio_thresh): m_ratio_thresh(ratio_thresh), m_method(method)
{
    if(m_method == Method::SIFT)
    {
       m_matcher_sift = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_SL2);
    }

    if(m_method == Method::ORB)
    {
        m_matcher_orb = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
    }

    if(m_method == Method::BRISK)
    {
        m_matcher_brisk = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
    }
}


Matcher::~Matcher()
{

}


void Matcher::Match(const cv::Mat& descriptor_object, const cv::Mat& descriptor_scene,
                    const std::vector<cv::KeyPoint>& keypoints_scene, std::vector<cv::DMatch>& good_matches_filtered)
{
    try
    {
        good_matches_filtered.clear();                      //clear from previous iteration
        std::vector< std::vector<cv::DMatch>> knn_matches;
        if((!descriptor_object.empty() && !descriptor_scene.empty()))
        {
            if(m_method == Method::SIFT)
            {
                m_matcher_sift->knnMatch(descriptor_object, descriptor_scene, knn_matches, 2);
            }

            if(m_method == Method::ORB)
            {
                m_matcher_orb->knnMatch(descriptor_object, descriptor_scene, knn_matches, 2);
            }

            if(m_method == Method::BRISK)
            {
                m_matcher_brisk->knnMatch(descriptor_object, descriptor_scene, knn_matches, 2);
            }
        }

        if(!knn_matches.empty() && !keypoints_scene.empty())
        {
            FilterOutliersLoweTest(knn_matches, good_matches_filtered);
            //std::vector<cv::DMatch> good_matches;
            //FilterOutliersLoweTest(knn_matches, good_matches);
            //good_matches_filtered = good_matches;
            //FilterOutliersSpatial(keypoints_scene, good_matches, good_matches_filtered);
        }
    }
    catch(cv::Exception& e)
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}


void Matcher::FilterOutliersLoweTest(std::vector< std::vector<cv::DMatch>>& knn_matches, std::vector<cv::DMatch>& good_matches)
{
    for(size_t i = 0; i < knn_matches.size(); i++)
    {
        if(knn_matches[i][0].distance < m_ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
}


void Matcher::DistanceFromCentroid(const std::vector<cv::Point2f>& points, cv::Point2f centroid, std::vector<double>& distances)
{
    std::vector<cv::Point2f>::const_iterator point;
    for(point = points.begin(); point != points.end(); ++point)
    {
        double distance = std::sqrt((point->x - centroid.x) * (point->x - centroid.x) + (point->y - centroid.y) * (point->y - centroid.y));
        distances.push_back(distance);
    }
}


void Matcher::FilterOutliersSpatial(const std::vector<cv::KeyPoint>& keypoints_scene, std::vector<cv::DMatch>& good_matches, std::vector<cv::DMatch>& good_matches_filtered)
{
    good_matches_filtered.clear();

    
    std::vector<cv::Point2f> points;

    //************************************* Centroid calculation *********************************************************
    for(size_t i = 0; i < good_matches.size(); i++)
    {
        points.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
    }
    cv::Moments moment = cv::moments(points, false);
    cv::Point2d centroid(moment.m10 / moment.m00, moment.m01 / moment.m00);

    //************************************* Spatial filtering *********************************************************

    std::vector<double> distances;
    DistanceFromCentroid(points, centroid, distances);

    cv::Scalar mu, sigma;
    if(!distances.empty())
    {
        cv::meanStdDev(distances, mu, sigma);
        std::vector<double>::iterator distance;
        for(size_t i = 0; i < distances.size(); ++i)
        {
            if(distances[i] < (mu.val[0] + 2.0 * sigma.val[0]))
            {
                good_matches_filtered.push_back(good_matches[i]);
            }
        }
    }
}


