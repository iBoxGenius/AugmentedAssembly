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

constexpr int PRINT_INST = 0;

class AugmentedInstructions
{
public:

    /**
     * @brief Constructor
     * 
     * @param corners Corners of all the objects sides 
     * @param camera_image_left Reference to the left camera frame
     * @param camera_image_right Reference to the right camera frame
     * @param assembly_state Reference to the state of the assembly
     * @param step_indices Reference to the step_indices (which part is active)
     * @param parts_cnt Total parts count
     * @param keypoints_size Reference to the keypoints size (for display)
     */
    AugmentedInstructions(std::vector<std::vector<std::vector<cv::Point>>>& corners, cv::Mat& camera_image_left, cv::Mat& camera_image_right,
                            AssemblyStates& assembly_state, std::vector<unsigned>& step_indices, unsigned& parts_cnt, int& keypoints_size);
    ~AugmentedInstructions();

    /**
     * @brief Starts the instructions process. Runs in a thread
     * 
     */
    void StartInstructions();


    /**
     * @brief Returns the left augmented frame as a char array
     *
     * @return char array of the left augmented image
     */
    char* GetLeftFrame();

    /**
     * @brief Returns the right augmented frame as a char array
     *
     * @return char array of the right augmented image
     */
    char* GetRigthFrame();

private:
    int& m_keypoints_size;  //display of the KPs in the scene

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

    std::vector<std::vector<std::vector<cv::Point>>>& m_corners;
    cv::Mat& m_image_camera_left;
    cv::Mat& m_image_camera_right;

    cv::Mat m_image_text_video;
    cv::Mat m_image_show;

    bool m_found_parts = false;
    std::vector<unsigned> m_found_part_cnt;

    unsigned m_steps_cnt = 0;
    unsigned m_steps_current_step = 1;
    std::vector<unsigned>& m_step_indices;
    std::vector<std::string> m_written_instructions;

    std::vector<AugmentedInstructions::Sides> m_sides_to_match;  //[0] -> first component; [1] -> second component. In the order, which the objects are defined/parsed

    /**
     * @brief Determines whether the sides are close
     * 
     * @return
     */
    bool AreSidesClose();

    /**
     * @brief Calculates the center of a component's side to connect
     * 
     * @param which_step_component
     * @param center [out]
     */
    void CalculateCenter(const unsigned which_step_component, cv::Point& center);

    /**
     * @brief Calculates the angle of a component's points, which form a line, to connect
     * 
     * @param which_step_component
     * @param angle [out]
     */
    void CalculateAngle(const unsigned& which_step_component, double& angle);

    /**
     * @brief Checks if the components are connected, meaning if the 'to connect' side is inside the 'to connect' plane
     * 
     * @param center Center of the first component
     * @param which_step_component Which component to connect to
     * @return 
     */
    bool IsInsideComponent(const cv::Point& center, const unsigned& which_step_component);

    /**
     * @brief Determines if the component is inside the allowed slice of the objects plane
     * 
     * @param center_to_test Center to test
     * @param center_from Center to form the slice from
     * @param which_step_component
     * @return 
     */
    bool IsInsideSlice(const cv::Point& center_to_test, const cv::Point& center_from, const unsigned& which_step_component);

    /**
     * @brief Checks if the angle is within the limit
     * 
     * @param angle_1
     * @param angle_2
     * @return 
     */
    inline bool IsAngleOk(const double angle_1, const double angle_2)
    {
        return (abs(angle_1 - angle_2) < 25);
    }
    bool m_draw_slice = false;
    std::vector<AugmentedInstructions::Object_match> m_object_step_properties;

    /**
     * @brief Sets the next objects to start detecting
     * 
     */
    void SetAssemblyIndices();
    UINT_PTR m_timer_id;
    unsigned m_timer_duration_ms;
    unsigned m_detected_max;

    /**
     * @brief Sets and resets parameters for a transition
     * 
     * @param timer_dur Delay of transition
     * @param detect_cnt How many detections needed to transition to the next state
     */
    void SetForNextState(unsigned timer_dur, unsigned detect_cnt);

    /**
     * @brief Callback function used after the timeout
     * 
     */
    void SetNextStateCallback();

    /**
     * @brief Checks for a new state in the main loop
     * 
     */
    void CheckForNewState();


    /**
     * @brief Sets the pointer to self (Used for the Windows API)
     * 
     * @param ptr Self
     */
    static void SetThisPtr(AugmentedInstructions* ptr)
    {
        this_ptr = ptr;
    }

    static AugmentedInstructions* GetThisPtr()
    {
        return this_ptr;
    }
    static AugmentedInstructions* this_ptr;

    /**
     * @brief Draws the necessary instructions
     * 
     */
    void DrawInstructions();

    /**
     * @brief Draws the labels of the detected objects
     *  
     * @param corners Corners of the object
     * @param index Which component it is
     */
    void DrawLabel(const std::vector<cv::Point>& corners, const unsigned index);

    /**
     * @brief Inserts the animation to the image
     * 
     */
    void InsertAnimation();

    /**
     * @brief Inserts the text to the image
     * 
     */
    void InsertText();

    /**
     * Determines if the correct planes of the objects for connection are visible in the image
     * 
     */
    void DetermineCorrectPlanes();

    /**
     * @brief Blinks the sides of the components to connect
     * 
     * @param which_component
     * @param corners Corners of the object's side
     */
    void BlinkSide(unsigned which_component, std::vector<cv::Point> corners);
    struct BlinkSides
    {
        UINT_PTR timer_blink_id = 0;
        unsigned current_cnt = 0;
        std::vector<cv::Point> pts;
        unsigned current_cycle = 0;
        unsigned max_cycles = 1;
        bool is_correct_plane = true;
        int visible_plane = -1;
        std::vector<unsigned>* connectable_planes = nullptr;
    };
    std::vector<BlinkSides> m_blink_sides;

    /**
     * @brief Callback function for a BlinkTimer
     * 
     * @param which_timer
     */
    void BlinkCallback(unsigned which_timer);
    std::vector<cv::VideoCapture> m_videos;

    /**
     * @brief Loads the animations from the path.
     * 
     * @param path_to_steps Aboslute path to the animations
     */
    void LoadStepAnimations(std::filesystem::path path_to_steps);

    cv::Point m_center;
    cv::Point m_pt1;
    cv::Point m_pt2;
    cv::Point m_pt3;
    cv::Point m_pt4;

    /**
     * 
     * 
     * @param points
     */
    void SortPointsCounterClockwise(std::vector<cv::Point>& points);

    unsigned m_fps = 0;
    UINT_PTR m_timer_fps_id;

    void FpsCallback();
    std::unique_ptr <char[]> m_image_left_unity;
    std::unique_ptr <char[]> m_image_right_unity;

    //************************ Callbacks for the Windows API *******************************//

    static VOID CALLBACK TimerProcStatic(HWND hwnd, UINT uMsg, UINT_PTR idEvent, DWORD dwTime)
    {
        AugmentedInstructions* pA = GetThisPtr();
        if(pA)
        {
            pA->SetNextStateCallback();
        }
    }

    static VOID CALLBACK TimerFpsStatic(HWND hwnd, UINT uMsg, UINT_PTR idEvent, DWORD dwTime)
    {
        AugmentedInstructions* pA = GetThisPtr();
        if(pA)
        {
            pA->FpsCallback();
        }
    }

    static VOID CALLBACK TimerBlink0Static(HWND hwnd, UINT uMsg, UINT_PTR idEvent, DWORD dwTime)
    {
        AugmentedInstructions* pA = GetThisPtr();
        if(pA)
        {
            pA->BlinkCallback(0);
        }
    }

    static VOID CALLBACK TimerBlink1Static(HWND hwnd, UINT uMsg, UINT_PTR idEvent, DWORD dwTime)
    {
        AugmentedInstructions* pA = GetThisPtr();
        if(pA)
        {
            pA->BlinkCallback(1);
        }
    }
};
