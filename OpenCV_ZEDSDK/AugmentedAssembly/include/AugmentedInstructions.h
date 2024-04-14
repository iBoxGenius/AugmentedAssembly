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


    void InsertAnimation();
    bool HasStateChanged();

private:

    AssemblyStates m_assembly_state;
    unsigned& m_parts_cnt;
    std::mutex& m_mutex;                                      //mutexe used for notifying the main thread that new object locations have been updated

    std::vector<std::vector<std::vector<cv::Point>>>& m_corners;
    cv::Mat& m_image_camera;
    cv::Mat m_image_text_video;
    cv::Mat m_image_show;

    bool m_found_parts = false;
    std::vector<unsigned> m_found_part_cnt;
    unsigned 

    unsigned m_steps_cnt = 0;
    unsigned m_steps_current_step = 1;
    std::vector<unsigned>& m_step_indices;
    std::mutex m_mutex_instructions;
    bool m_changed_state = true;
    bool m_changed_state_for_main = true;

    enum class Sides
    {
        Top,
        Right,
        Bottom,
        Left
    };

    std::vector<AugmentedInstructions::Sides> m_sides_to_match;  //[0] -> first component; [1] -> second component. In the order, which the objects are defined/parsed
    bool AreSidesClose();

    void SetAssemblyIndices();
    HWND m_window_handle;
    UINT_PTR m_timer_id;
    unsigned m_timer_duration_ms;
    unsigned m_detected_max;

    void SetForNextState(unsigned timer_dur, unsigned detect_cnt);

    static VOID CALLBACK TimerProcStatic(HWND hwnd, UINT uMsg, UINT_PTR idEvent, DWORD dwTime)
    {
        AugmentedInstructions* pA = GetThisPtr();
        if(pA)
            pA->SetNextStateCallback();
    }
    void SetNextStateCallback();
    void CheckForNewState();

    static void SetThisPtr(AugmentedInstructions* ptr) {
        this_ptr = ptr;
    }

    static AugmentedInstructions* GetThisPtr() {
        return this_ptr;
    }
    static AugmentedInstructions* this_ptr;


    void DrawInstructions();
    void DrawLabel(const std::vector<cv::Point>& corners, Sides& lines, const unsigned index);

    std::vector<cv::VideoCapture> m_videos;
    void LoadStepAnimations(std::filesystem::path path_to_steps);
};