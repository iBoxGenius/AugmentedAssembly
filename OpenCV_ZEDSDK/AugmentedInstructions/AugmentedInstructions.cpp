#include "AugmentedInstructions.h"

#include <windows.h>


AugmentedInstructions::AugmentedInstructions(std::vector<std::vector<cv::Point>>& corners, cv::Mat& image): m_corners(corners), m_image_clean(image)
{
    m_delay = std::chrono::milliseconds(150);
    m_blink = false;
}

AugmentedInstructions::~AugmentedInstructions()
{
}



/*
void AugmentedInstructions::StartTimer(std::vector<std::vector<cv::Point2f>>& corners)
{
    if(!m_threadRunning)
    {
        m_threadRunning = true;
        m_thread = std::thread([this]()
        {
            while(m_threadRunning) 
            {
                std::this_thread::sleep_for(m_delay);
                if(m_blink) 
                {
                    m_callback();
                }
            }
        });
        m_thread.detach();
    }
}
*/

void AugmentedInstructions::StartBlinkTimer()
{
    auto end_time = std::chrono::high_resolution_clock::now();
    auto start_time = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    bool blink_timer = true;
    unsigned blink_cnt = 0;
    bool blink_colour = true;

    while(true)
    {
        //m_image.copyTo(m_image_clean);
        /*
        if(m_blink)
        {
            if(blink_timer)
            {
                start_time = std::chrono::high_resolution_clock::now();
                blink_timer = false;
                m_image.copyTo(m_image_clean);
            }

            if(!blink_timer)
            {
                end_time = std::chrono::high_resolution_clock::now();
                dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

                if(dur >= m_delay)
                {
                    if(blink_cnt < 6)
                    {
                        BlinkPlanes(blink_colour);
                        blink_colour = !blink_colour;
                        blink_cnt++;
                        start_time = std::chrono::high_resolution_clock::now();
                    }
                    else
                    {
                        std::cout << "Finished" << std::endl;
                        m_blink = false;
                        blink_cnt = 0;
                        blink_timer = true;
                        blink_colour = true;
                    }
                }
            }
        }
        */
    }

}


void AugmentedInstructions::BlinkPlanes()
{
    //m_blink = true;
    m_blink_rq = true;
}

void AugmentedInstructions::BlinkPlanesWin()
{
    if(m_blink_rq)
    {
        if(m_blink_img_first)
        {
            m_image_draw.copyTo(image_clean_blink);
            m_blink_img_first = false;
        }

        if(m_blink_img_cnt < 6)
        {
            if(m_blink_win)
            {
                for(auto& corners : m_corners)
                {
                    cv::fillConvexPoly(m_image_draw, corners.data(), corners.size(), cv::Scalar(0, 255, 0));
                }
                std::cout << "Coloured" << std::endl;
                m_blink_win = false;
            }
            else
            {
                m_image_clean.copyTo(m_image_draw);
                std::cout << "Clean" << std::endl;
                m_blink_win = true;
            }
        }
        else
        {
            //kill setTimer
            //KillTimer(hwnd, timerId);
        }
    }
}


void AugmentedInstructions::BlinkPlanes(bool colour)
{
    if(colour)
    {
        for(auto& corners : m_corners)
        {
            cv::fillConvexPoly(m_image, corners.data(), corners.size(), cv::Scalar(0, 255, 0));
        }
        std::cout << "Coloured" << std::endl;
    }
    else
    {
        m_image_clean.copyTo(m_image);
        std::cout << "Clean" << std::endl;
    }

    cv::imshow("Frame", m_image);
    cv::waitKey(10);
}

