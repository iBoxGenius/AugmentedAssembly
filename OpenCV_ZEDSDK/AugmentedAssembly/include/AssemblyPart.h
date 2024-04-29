#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <iostream>
#include <chrono>
#include <shared_mutex> 
#include <mutex> 
#include <thread> 
#include "Enums.hpp"
#include "Matcher.h"

#include <Windows.h>

class AssemblyPart {
public:
    //AssemblyPart(Method method, std::filesystem::path path_to_images, std::filesystem::path path_to_json, std::mutex& mutex, std::atomic<bool>& sync_var);
    AssemblyPart(Method method, std::filesystem::path path_to_images, std::filesystem::path path_to_json, std::mutex& mutex, uint8_t& sync_var, std::condition_variable_any& cv);
    ~AssemblyPart();                // Destructor

    void SetDescriptors(std::vector<cv::Mat>& desc);
    std::vector<cv::Mat> GetDescriptors();
    std::vector<cv::Mat> GetImages();
    std::vector<std::vector<cv::KeyPoint>> GetKeypoints();
    std::vector<std::vector<cv::DMatch>> GetFilteredMatches();
    std::vector<cv::KeyPoint> GetKpSceneCopy();

    void SetNewSceneParam(cv::Mat descriptor_scene, std::vector<cv::KeyPoint> keypoints_scene);
    //void FindMatches(const cv::Mat& descriptor_scene, const std::vector<cv::KeyPoint>& keypoints_scene);
    //Thread function
    void FindMatches(const cv::Mat& descriptor_scene, const std::vector<cv::KeyPoint>& keypoints_scene, std::vector<std::vector<cv::Point>>& scene_corners);

    inline HANDLE GetThreadHandle()
    {
        return GetCurrentThread();
    }

private:
    const size_t iID;
    static size_t iLiving;
    static size_t iTotal;

    std::vector<cv::Mat> m_images;
    std::vector<std::vector<cv::KeyPoint>> m_keypoints;       //keypoints per image
    std::vector<cv::Mat> m_descriptors;

    std::mutex& m_mutex;                                      //mutexes used for notifying the main thread that new object locations have been updated
    //std::atomic<bool>& m_new_kp_rq;
    uint8_t& m_new_kp_rq;
    cv::Mat m_descriptor_scene_local_cpy;
    std::vector<cv::KeyPoint> m_keypoints_scene_local_cpy;    //std::vector

    std::condition_variable_any& m_cv;

    cv::Ptr<cv::DescriptorMatcher> m_matcher_sift;
    cv::Ptr<cv::DescriptorMatcher> m_matcher_orb;
    cv::Ptr<cv::DescriptorMatcher> m_matcher_brisk;
    Method m_method;
    void LoadKeypointsFromImgs(std::filesystem::path path_to_images, std::filesystem::path path_to_json);
    bool isRectangularShape(std::vector<cv::Point2f>& pts);

    std::vector<Matcher> m_matchers;
    std::vector<std::vector<cv::DMatch>> m_good_matches_filtered;
    //void FindBestMatch();
};
