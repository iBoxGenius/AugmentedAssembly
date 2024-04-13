#pragma once

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
#include "Enums.hpp"
#include <Windows.h>

#include <filesystem>

class AugmentedInstructions {
public:
    AugmentedInstructions(std::vector<std::vector<std::vector<cv::Point>>>& corners, cv::Mat& camera_image, std::mutex& mutex, AssemblyStates& assembly_state, std::vector<unsigned>& step_indices, unsigned& parts_cnt);
    ~AugmentedInstructions();                // Destructor

    void StartInstructions();


    void InsertAnimation(cv::Mat &camera_frame);

private:

    AssemblyStates m_assembly_state;
    unsigned& m_parts_cnt;
    std::mutex& m_mutex;                                      //mutexes used for notifying the main thread that new object locations have been updated

    std::vector<std::vector<std::vector<cv::Point>>>& m_corners;
    cv::Mat& m_image_camera;
    cv::Mat m_image_text_video;
    cv::Mat m_image_show;

    bool m_found_parts = false;
    std::vector<unsigned> m_found_part_cnt;

    unsigned m_steps_cnt = 0;
    std::vector<unsigned>& m_step_indices;
    std::mutex m_mutex_instructions;
    bool m_changed_state = true;
    void SetAssemblyIndices();
    HWND m_window_handle;
    UINT_PTR m_timer_id;

    void SetTimerNextState();

    static VOID CALLBACK TimerProcStatic(HWND hwnd, UINT uMsg, UINT_PTR idEvent, DWORD dwTime)
    {
        AugmentedInstructions* pA = GetThisPtr();
        if(pA)
            pA->SetStateStepCallback();
    }
    void SetStateStepCallback();

    static void SetThisPtr(AugmentedInstructions* ptr) {
        this_ptr = ptr;
    }

    static AugmentedInstructions* GetThisPtr() {
        return this_ptr;
    }
    static AugmentedInstructions* this_ptr;


    enum class Lines_label
    {
        Top,
        Right,
        Bottom,
        Left
    };


    void DrawInstructions();
    void DrawLabel(const std::vector<cv::Point>& corners, Lines_label& lines, const unsigned index);

    std::vector<cv::VideoCapture> m_videos;
    void LoadStepAnimations(std::filesystem::path path_to_steps);
};