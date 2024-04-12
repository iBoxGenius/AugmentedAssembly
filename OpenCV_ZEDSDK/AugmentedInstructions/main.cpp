

#include <iostream>
#include <sl/Camera.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <shared_mutex> 
#include <mutex> 
#include <thread> 

#include <chrono>

#include "AugmentedInstructions.h"

int main()
{   
    cv::Mat image = cv::imread("object_1_0.png", cv::IMREAD_COLOR);
    cv::Mat image_new;
    image.copyTo(image_new);

    std::vector<std::vector<cv::Point>> corners;
    std::vector<cv::Point> vec = {cv::Point(100,100), cv::Point(300,100), cv::Point(300,300), cv::Point(100,300) };
    corners.push_back(vec);

    AugmentedInstructions instructions(corners, image);
    auto thread_instructions = std::thread(&AugmentedInstructions::StartBlinkTimer, std::ref(instructions));
    while(true)
    {
        //instructions.BlinkPlanes();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        imshow("Frame", image);
        //cv::waitKey(40);

        image_new.copyTo(image);

    }


    return 0;
}
