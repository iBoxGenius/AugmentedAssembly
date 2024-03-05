
#include <iostream>
#include <sl/Camera.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/flann.hpp>


#include <string>

inline cv::Mat slMat2cvMat(sl::Mat& input) {
    int cv_type = -1;
    cv::Mat ret;
    switch(input.getDataType()) {
    case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1;
        break;
    case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2;
        break;
    case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3;
        break;
    case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4;
        break;
    case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1;
        break;
    case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2;
        break;
    case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3;
        break;
    case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4;
        break;
    default: break;
    }

    auto xd = input.getPtr<sl::uchar1>(sl::MEM::CPU);
    if(xd != NULL)
    {
        return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
    }
    return ret;
}


void IncreaseFocus(sl::Camera& zed, unsigned sharpness)
{
    zed.setCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS, sharpness);
}






/*********************************  Unified Camera initializer    *******************************************************/
void InitCamera(sl::Camera& zed)
{
    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_parameters.camera_fps = 30;
    auto returned_state = zed.open(init_parameters);
    zed.setCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS, 5);
    if(returned_state != sl::ERROR_CODE::SUCCESS) {
        std::cout << "Camera error: " << returned_state << std::endl;
    }
}

int main()
{
    sl::Camera zed;
    InitCamera(zed);

    sl::Mat grabbed_frame;
    cv::Mat cv_frame;

    cv::Mat keypoints_frame;
    cv::Mat demonstration_frame;

    cv::Ptr<cv::BRISK> detector_brisk = cv::BRISK::create();
    cv::Ptr<cv::ORB> detector_orb = cv::ORB::create();
    cv::Ptr<cv::SIFT> detector_sift = cv::SIFT::create();
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptor;

    auto start_time = std::chrono::high_resolution_clock::now();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();


    while(true)
    {
        auto returned_state = zed.grab();
        if(returned_state == sl::ERROR_CODE::SUCCESS)
        {
            zed.retrieveImage(grabbed_frame, sl::VIEW::LEFT, sl::MEM::CPU);
            cv_frame = slMat2cvMat(grabbed_frame);
        }


        demonstration_frame = cv_frame.clone();

        detector_brisk->detectAndCompute(demonstration_frame, cv::noArray(), keypoints, descriptor, false);

        auto kp_cpy = keypoints;
        cv::KeyPointsFilter::removeDuplicated(kp_cpy);
        cv::KeyPointsFilter::retainBest(keypoints, keypoints.size() * 0.8);
          
        //cv::drawKeypoints(demonstration_frame, keypoints, demonstration_frame);
        /*
        std::sort(keypoints.begin(), keypoints.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return (a.response > b.response); });
        std::vector<cv::KeyPoint> keypoints_strong = std::vector<cv::KeyPoint>(keypoints.begin(), keypoints.begin() + (keypoints.size() / 2));
        */
        std::vector<cv::KeyPoint> keypoints_strong = keypoints;

        start_time = std::chrono::high_resolution_clock::now();

        cv::Mat keypointMat(keypoints_strong.size(), 2, CV_32FC1);

        // Fill the matrix with keypoints' (x, y) coordinates
        for(size_t i = 0; i < keypoints_strong.size(); ++i) {
            keypointMat.at<float>(i, 0) = keypoints_strong[i].pt.x;
            keypointMat.at<float>(i, 1) = keypoints_strong[i].pt.y;
        }


        int numClusters = 4;
        cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0);

        cv::Mat labels, centers;
        auto compactness = cv::kmeans(keypointMat, numClusters, labels, criteria, 25, cv::KMEANS_PP_CENTERS, centers);
        
        end_time = std::chrono::high_resolution_clock::now();
        dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        std::cout << "Detector: " << dur << "ms" << "\t" << descriptor.size() << std::endl;
        std::cout << "Compactness: " << compactness/(keypoints.size()^2) << std::endl;
        std::cout << "-----------------------------" << std::endl;

        labels.reshape(1);

        //std::this_thread::sleep_for(std::chrono::milliseconds(1400));

        for(size_t i = 0; i < labels.rows; i++)
        {
            if(labels.at<unsigned>(i) == 0)
            {
                cv::circle(demonstration_frame, cv::Point(keypointMat.at<float>(i, 0), keypointMat.at<float>(i, 1)), 10, cv::Scalar(0, 0, 255), 2);
            }
            if(labels.at<unsigned>(i) == 1)
            {
                cv::circle(demonstration_frame, cv::Point(keypointMat.at<float>(i, 0), keypointMat.at<float>(i, 1)), 10, cv::Scalar(0, 255, 0), 2);
            }

            if(labels.at<unsigned>(i) == 2)
            {
                cv::circle(demonstration_frame, cv::Point(keypointMat.at<float>(i, 0), keypointMat.at<float>(i, 1)), 10, cv::Scalar(255, 0, 0), 2);
            }


            if(labels.at<unsigned>(i) == 3)
            {
                cv::circle(demonstration_frame, cv::Point(keypointMat.at<float>(i, 0), keypointMat.at<float>(i, 1)), 10, cv::Scalar(255, 255, 255), 2);
            }
        }
        //
        /*
        for(size_t i = 0; i < 30; i++)
        {
            std::cout << keypoints[i].pt.x << "," << keypoints[i].pt.y << std::endl;
            std::cout << keypointMat.at<float>(i, 0) << "," << keypointMat.at<float>(i, 1) << std::endl;
            std::cout << "---------------" << std::endl;
        }
        */

        cv::waitKey(20);
        cv::imshow("Camera feed", demonstration_frame);

        keypoints.clear();
        keypoints_strong.clear();
        descriptor.release();
    }
}

