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

#include <filesystem>
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
    cv::Mat demonstration_frame;

    std::vector<cv::VideoCapture> m_videos(1);
    auto path_to_video = std::filesystem::path(std::filesystem::current_path() / ("resources/steps")).make_preferred();
    for(auto& p : std::filesystem::directory_iterator(path_to_video))
    {
        //std::cout << p.path() << '\n';
        if(p.path().extension() == ".avi")
            for(size_t i = 0; i < 1; i++)       //for each step
            {
                std::filesystem::path tmp = p.path();
                cv::String str(tmp.string());
                //m_videos.push_back(cv::VideoCapture(str));
                //cv::VideoCapture cap = cv::VideoCapture(str);
                m_videos[0].open(str, cv::CAP_FFMPEG);
            }
    }

    //cv::VideoCapture cap;
    if(m_videos[0].isOpened() == 0)
    {
        std::cout << "The video file cannot be opened." << std::endl;
        return -1;
    }

    while(true)
    {
        auto returned_state = zed.grab();
        if(returned_state == sl::ERROR_CODE::SUCCESS)
        {
            zed.retrieveImage(grabbed_frame, sl::VIEW::LEFT, sl::MEM::CPU);
            cv_frame = slMat2cvMat(grabbed_frame);

            demonstration_frame = cv_frame.clone();
        }

        
        cv::Mat video;
        m_videos[0] >> video;


        cv::waitKey(30);
        cv::imshow("Camera feed", demonstration_frame);
    }

    return 0;
}
