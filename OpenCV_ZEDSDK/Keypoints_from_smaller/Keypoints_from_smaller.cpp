#include <iostream>
#include <sl/Camera.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

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

    cv::String desc_name = "desriptor_";
    cv::String kp_name = "keypoints_";
    unsigned desc_iter = 0;
    unsigned focus = 0;
   
    std::vector<cv::Mat> object_images;
    object_images.reserve(6);

    for(size_t i = 0; i < 6; i++)
    {
        object_images.push_back(cv::imread("object_0_" + std::to_string(i) + ".png", cv::IMREAD_GRAYSCALE));
        keypoints.clear();
        descriptor.release();
        //detector, descriptor creation
        keypoints_frame = object_images[i].clone();
        detector_brisk->detectAndCompute(keypoints_frame, cv::noArray(), keypoints, descriptor, false);
        cv::drawKeypoints(keypoints_frame, keypoints, keypoints_frame);

        /*********************************  Writing the descriptor    *******************************************************/

        std::string object_name = "object_0";
        cv::FileStorage store_desc(("descriptor_" + object_name + ".json"), cv::FileStorage::APPEND);
        cv::write(store_desc, desc_name + std::to_string(desc_iter), descriptor);
        store_desc.release();

        cv::FileStorage store_kp(("keypoints_" + object_name + ".json"), cv::FileStorage::APPEND);
        cv::write(store_kp, kp_name + std::to_string(desc_iter), keypoints);
        store_kp.release();


        desc_iter++;

        /********************************************************************************************************************/

        cv::imshow("Keypoints BRISK", keypoints_frame);
        cv::waitKey(0);  //
    }
}
