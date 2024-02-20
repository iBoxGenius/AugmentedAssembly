
#include <iostream>
#include <sl/Camera.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>


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


/*********************************  Unified Camera initializer    *******************************************************/
void InitCamera(sl::Camera& zed)
{
    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_parameters.camera_fps = 30;
    zed.setCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS, 5);
    auto returned_state = zed.open(init_parameters);
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
    cv::Mat resized_frame;

    cv::Ptr<cv::BRISK> detector_brisk = cv::BRISK::create();
    cv::Ptr<cv::ORB> detector_orb = cv::ORB::create();
    cv::Ptr<cv::SIFT> detector_sift = cv::SIFT::create();
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptor;

    auto start_time = std::chrono::high_resolution_clock::now();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();


    
    /*********************************  Reading the descriptor    *******************************************************/
    /*
    cv::Mat descriptor_read;
    cv::FileStorage store("descriptor_BRISK.txt", cv::FileStorage::READ);
    cv::FileNode n2 = store["descriptors"];
    cv::read(n2, descriptor_read);
    store.release();
    */
    /********************************************************************************************************************/
    cv::String desc_name = "desriptor_";
    unsigned desc_iter = 0;
    while(true)
    {
        returned_state = zed.grab();
        if(returned_state == sl::ERROR_CODE::SUCCESS)
        {
            zed.retrieveImage(grabbed_frame, sl::VIEW::LEFT, sl::MEM::CPU);
            cv_frame = slMat2cvMat(grabbed_frame);
        }

        cv::imshow("Camera feed", cv_frame);
        if((char)cv::waitKey(30) == 'q')
        {

            std::cout << "Snapshot taken [BRISK]: ";
            //detector, descriptor creation
            keypoints_frame = cv_frame.clone();
            start_time = std::chrono::high_resolution_clock::now();

            detector_brisk->detectAndCompute(keypoints_frame, cv::noArray(), keypoints, descriptor, false);

            end_time = std::chrono::high_resolution_clock::now();
            dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            std::cout << dur << " ms" << std::endl;

            cv::drawKeypoints(keypoints_frame, keypoints, keypoints_frame);

            /*********************************  Writing the descriptor    *******************************************************/
            

            cv::FileStorage store("descriptor_BRISK_Eiffel_next.json", cv::FileStorage::APPEND);
            cv::write(store, desc_name + std::to_string(desc_iter), descriptor);
            store.release();
            desc_iter++;
            cv::imwrite("image1.jpg", cv_frame);
            /********************************************************************************************************************/

            cv::imshow("Keypoints BRISK", keypoints_frame);
            cv::waitKey(2000);  //
        }

        if((char)cv::waitKey(30) == 'w')
        {

            std::cout << "Snapshot taken [ORB]: ";
            //detector, descriptor creation
            cv_frame.copyTo(keypoints_frame);
            start_time = std::chrono::high_resolution_clock::now();

            detector_orb->detectAndCompute(keypoints_frame, cv::noArray(), keypoints, descriptor, false);

            end_time = std::chrono::high_resolution_clock::now();
            dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            std::cout << dur << " ms" << std::endl;

            cv::drawKeypoints(keypoints_frame, keypoints, keypoints_frame);
            cv::imshow("Keypoints ORB", keypoints_frame);
            cv::waitKey(2000);  //
        }

        if((char)cv::waitKey(30) == 'e')
        {

            std::cout << "Snapshot taken [SIFT]: ";
            //detector, descriptor creation
            cv_frame.copyTo(keypoints_frame);
            start_time = std::chrono::high_resolution_clock::now();

            detector_sift->detectAndCompute(keypoints_frame, cv::noArray(), keypoints, descriptor, false);

            end_time = std::chrono::high_resolution_clock::now();
            dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            std::cout << dur << " ms" << std::endl;

            cv::drawKeypoints(keypoints_frame, keypoints, keypoints_frame);
            cv::imshow("Keypoints SIFT", keypoints_frame);
            cv::waitKey(2000);  //
        }

    }
}
