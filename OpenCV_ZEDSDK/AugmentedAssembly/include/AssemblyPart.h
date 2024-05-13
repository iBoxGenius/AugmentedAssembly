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

constexpr int PRINT = 0;


class AssemblyPart {
public:
   /**
    * @brief Constructor
    * 
    * @param method The method for keypoints/descriptor
    * @param path_to_images Absolute path to the folder with the images
    * @param path_to_json Absolute path to the folder with the .json files (descriptor, keypoints)
    * @param mutex  mutex to protect the request flag (sync_var)
    * @param sync_var  states the request status of the AssemblyPart 
    * @param cv conditional variable used to run/suspend the thread
    */
    AssemblyPart(Method method, std::filesystem::path path_to_images, std::filesystem::path path_to_json, std::mutex& mutex, uint8_t& sync_var, std::condition_variable_any& cv);
    ~AssemblyPart();

    /**
     * @brief Provides a new set of descriptor and keypoints. Creates local copies
     * 
     * @param descriptor_scene New descriptor of the scene
     * @param keypoints_scene New keypoints of the scene
     */
    void SetNewSceneParam(cv::Mat descriptor_scene, std::vector<cv::KeyPoint> keypoints_scene);
    //Thread function

    /**
     * @brief Runs in a thread. Finds the match of the object's side and updates its corner locations in the scene
     * 
     * @param descriptor_scene Descriptor of the scene
     * @param keypoints_scene Keypoints of the scene
     * @param scene_corners Transformed corner locations to the scene
     */
    void FindMatches(const cv::Mat& descriptor_scene, const std::vector<cv::KeyPoint>& keypoints_scene, std::vector<std::vector<cv::Point>>& scene_corners);

private:
    const size_t iID;
    static size_t iLiving;
    static size_t iTotal;

    std::vector<cv::Mat> m_images;
    std::vector<std::vector<cv::KeyPoint>> m_keypoints;       //keypoints per image
    std::vector<cv::Mat> m_descriptors;

    std::mutex& m_mutex;                                      //mutexes used for notifying the main thread that new object locations have been updated
    uint8_t& m_new_kp_rq;                                     
    cv::Mat m_descriptor_scene_local_cpy;
    std::vector<cv::KeyPoint> m_keypoints_scene_local_cpy;
    std::condition_variable_any& m_cv;

    std::vector<Matcher> m_matchers;
    std::vector<std::vector<cv::DMatch>> m_good_matches_filtered;
    Method m_method;

    /**
     * @brief Loads the images, keypoints, descriptor from the folders
     * 
     * @param path_to_images Absolute path to the folder with the images
     * @param path_to_json Absolute path to the folder with the .json files (descriptor, keypoints)
     */
    void LoadKeypointsFromImgs(std::filesystem::path path_to_images, std::filesystem::path path_to_json);

    /**
     * @brief Calculates the angle of points that form a quad.
     * 
     * @param pts Points to be verified
     * @return True if OK, otherwise false
     */
    bool isRectangularShape(std::vector<cv::Point2f>& pts);
};
