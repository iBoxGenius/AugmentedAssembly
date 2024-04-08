#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <iostream>
#include <chrono>
#include <thread>
#include "Enums.hpp"
#include "Matcher.h"

#include <Windows.h>

class AssemblyPart {
public:
    AssemblyPart(Method method, std::filesystem::path path_to_images, std::filesystem::path path_to_json, std::mutex& mutex, std::atomic<bool>& sync_var);
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

private:
    const size_t iID;
    static size_t iLiving;
    static size_t iTotal;

    std::vector<cv::Mat> m_images;
    std::vector<std::vector<cv::KeyPoint>> m_keypoints;       //keypoints per image
    std::vector<cv::Mat> m_descriptors;

    std::mutex& m_mutex;                                      //mutexes used for notifying the main thread that new object locations have been updated
    std::atomic<bool>& m_new_kp_rq;
    cv::Mat m_descriptor_scene_local_cpy;
    std::vector<cv::KeyPoint> m_keypoints_scene_local_cpy;    //std::vector

    cv::Ptr<cv::DescriptorMatcher> m_matcher_sift;
    cv::Ptr<cv::DescriptorMatcher> m_matcher_orb;
    cv::Ptr<cv::DescriptorMatcher> m_matcher_brisk;
    Method m_method;
    void LoadKeypointsFromImgs(std::filesystem::path path_to_images, std::filesystem::path path_to_json);

    std::vector<Matcher> m_matchers;
    std::vector<std::vector<cv::DMatch>> m_good_matches_filtered;
    //void FindBestMatch();
};


/*  GOING OVER ALL THE REFERENCE IMAGESE
for i, reference_image in enumerate([img_reference1, img_reference2]):
    # Detect keypoints and extract descriptors for the reference image
    keypoints_reference, descriptors_reference = detector.detectAndCompute(reference_image, None)

    # Match descriptors using KNN
    matches = bf.knnMatch(descriptors_query, descriptors_reference, k=2)

    # Apply ratio test
    good_matches = []
    for m, n in matches:
        if m.distance < 0.75 * n.distance:
            good_matches.append(m)

    # Print the number of good matches for each reference image
    print(f"Number of good matches for reference image {i+1}: {len(good_matches)}")

    # Optionally, you can store or analyze the good matches further
    # For example, you might want to compute an overall score based on the number of good matches

    # Determine the best response based on your specific criteria
    threshold = 10  # Adjust this threshold based on your application
    if len(good_matches) > threshold:
        print(f"Query image is similar to reference image {i+1}")
    else:
        print(f"Query image is not similar to reference image {i+1}")

*/