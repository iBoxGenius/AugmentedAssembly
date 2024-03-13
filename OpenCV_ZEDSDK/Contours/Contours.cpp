
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


constexpr unsigned THRESHOLD_SEG = 128;
constexpr double THRESHOLD_CONTOUR_AREA = 2000;

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
    cv::Mat contour_frame;

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
        contour_frame = cv_frame.clone();
        cv::cvtColor(demonstration_frame, contour_frame, cv::COLOR_BGR2GRAY);
        detector_brisk->detectAndCompute(contour_frame, cv::noArray(), keypoints, descriptor, false);


        auto kp_cpy = keypoints;
        cv::KeyPointsFilter::removeDuplicated(kp_cpy);
        cv::KeyPointsFilter::retainBest(kp_cpy, kp_cpy.size() * 0.8);

        //cv::drawKeypoints(demonstration_frame, keypoints, demonstration_frame);
        /*
        std::sort(keypoints.begin(), keypoints.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return (a.response > b.response); });
        std::vector<cv::KeyPoint> keypoints_strong = std::vector<cv::KeyPoint>(keypoints.begin(), keypoints.begin() + (keypoints.size() / 2));
        */
        cv::threshold(contour_frame, contour_frame, THRESHOLD_SEG, 255, cv::THRESH_BINARY);
        cv::imshow("Threshold Image", contour_frame);


        start_time = std::chrono::high_resolution_clock::now();
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(contour_frame, contours, hierarchy,cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
        end_time = std::chrono::high_resolution_clock::now();
        dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        std::cout << "Contours: " << dur << "ms" << std::endl;

        cv::Mat contours_image = cv::Mat::zeros(contour_frame.size(), CV_8UC3);

        std::vector<std::vector<cv::Point> > contours_filtered;         // TO DO how to reserve
        for(int i = 0; i < contours.size(); i++)
        {
            double contourArea = cv::contourArea(contours[i]);
            if(contourArea > THRESHOLD_CONTOUR_AREA)
            {
                contours_filtered.push_back(contours[i]);
            }
        }

        cv::Mat image_bbs = demonstration_frame.clone();

        std::vector<unsigned> kp_idx(2000);           //idx == kp; [idx] == which contour
        //kp_idx.reserve(2000);
        start_time = std::chrono::high_resolution_clock::now();
        /*
        for(unsigned i = 0; i < contours_filtered.size(); i++)
        {
            for(unsigned j = 0; j < kp_cpy.size(); j++)
            {
                int flag = cv::pointPolygonTest(contours_filtered[i], kp_cpy[j].pt, false);
                if(flag >= 0)
                {
                    kp_idx[j] = i;
                }
            }
        }
        */

        for(unsigned i = 0; i < contours_filtered.size(); i++)
        {
            cv::Rect boundingBox = cv::boundingRect(contours_filtered[i]);
            cv::rectangle(image_bbs, boundingBox, cv::Scalar(0, 255, 0), 3);

            for(unsigned j = 0; j < kp_cpy.size(); j++)
            {
                int flag = boundingBox.contains(kp_cpy[j].pt);
                if(flag)
                {
                    kp_idx[j] = i;
                }
            }
        }
        end_time = std::chrono::high_resolution_clock::now();
        dur = std::chrono::duration_cast<std::chrono::microseconds > (end_time - start_time).count();
        std::cout << "KP filtering: " << dur << "us" << std::endl;


        for(size_t i = 0; i < contours_filtered.size(); i++)
        {
            cv::Scalar color = cv::Scalar(0, 255, 0);
            cv::drawContours(contours_image, contours_filtered, i, color, 2, cv::LINE_8, hierarchy, 0);
        }

        cv::waitKey(30);
        // Display the original and contours images
        cv::imshow("Original Image", demonstration_frame);
        cv::imshow("Contours Image", contours_image);
        cv::imshow("BBs Image", image_bbs);

        kp_idx.clear();
        keypoints.clear();
        kp_cpy.clear();
    }

}