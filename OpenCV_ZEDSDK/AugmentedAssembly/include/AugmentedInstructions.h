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

    enum class Sides
    {
        Top,
        Right,
        Bottom,
        Left
    };

    struct Object_match
    {
        std::vector<std::vector<AugmentedInstructions::Sides>> sides_to_match;
        std::vector<std::vector<unsigned>> planes_to_connect;
    };

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



    std::vector<AugmentedInstructions::Sides> m_sides_to_match;  //[0] -> first component; [1] -> second component. In the order, which the objects are defined/parsed
    bool AreSidesClose();
    void CalculateCenter(const unsigned which_step_component, cv::Point& center);
    void CalculateAngle(const unsigned& which_step_component, double& angle);
    bool IsInsideComponent(const cv::Point& center, const unsigned& which_step_component);
    bool IsInsideSlice(const cv::Point& center_to_test, const cv::Point& center_from, const unsigned& which_step_component);
    inline bool IsAngleOk(const double angle_1, const double angle_2)
    {
        return (abs(angle_1 - angle_2) < 25);
    }
    bool m_draw_slice = false;
    std::vector<std::vector<unsigned>> m_planes_to_connect;
    std::vector<AugmentedInstructions::Object_match> m_object_step_properties;
    

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


    cv::Point m_center;
    cv::Point m_pt1;
    cv::Point m_pt2;

    cv::Point m_pt3;
    cv::Point m_pt4;
};