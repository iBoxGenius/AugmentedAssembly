#include "AssemblyPart.h"

#include <omp.h>



size_t AssemblyPart::iLiving = 0;
size_t AssemblyPart::iTotal = 0;

double CalculateIoU(std::vector<cv::Point> pts_new, std::vector<cv::Point> pts_old)
{
    cv::Mat mask_new = cv::Mat::zeros(cv::Size(1280, 720), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours_new = { pts_new };
    cv::fillPoly(mask_new, contours_new, cv::Scalar(255));

    cv::Mat mask_old = cv::Mat::zeros(cv::Size(1280, 720), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours_old = { pts_old };
    cv::fillPoly(mask_old, contours_old, cv::Scalar(255));

    //AREA OF UNION
    cv::Mat mask_union = mask_new | mask_old;
    double area_of_union = cv::countNonZero(mask_union);

    //AREA OF INTERSECTION
    cv::Mat mask_intersection = cv::Mat::zeros(cv::Size(1280, 720), CV_8UC1);
    for(size_t j = 0; j < mask_new.rows; j++)
    {
        for(size_t i = 0; i < mask_new.cols; i++)
        {
            if(mask_old.at<uchar>(j, i) == 255 && mask_old.at<uchar>(j, i) == 255)
            {
                mask_intersection.at<uchar>(j, i) = 255;
            }
        }
    }
    double area_of_intersection = cv::countNonZero(mask_intersection);
    /*
    cv::imshow("Mask_intersection", mask_intersection);
    cv::waitKey(10);

    cv::imshow("Mask_new", mask_new);
    cv::waitKey(10);

    cv::imshow("Mask_old", mask_old);
    cv::waitKey(10);

    cv::imshow("Mask_union", mask_union);
    cv::waitKey(10);
    */
    double IoU = area_of_intersection / area_of_union;
    return IoU;
}


bool AssemblyPart::isRectangularShape(std::vector<cv::Point2f>& pts)
{
    std::vector<float> cosines;
    for(int i = 0; i < 4; ++i)
    {
        cv::Point2f v1 = pts[(i + 1) % 4] - pts[i];
        cv::Point2f v2 = pts[(i + 2) % 4] - pts[(i + 1) % 4];
        float dot = v1.x * v2.x + v1.y * v2.y;
        double magnitude = norm(v1) * norm(v2);
        float cosine = dot / magnitude;
        cosines.push_back(cosine);
    }

    //const double threshold = 0.25; // 
    const float threshold = 0.4f; //
    for(float cosine : cosines)
    {
        if(abs(cosine) > threshold)
        {
            return false;
        }
    }

    return true;
}

AssemblyPart::AssemblyPart(Method method, std::filesystem::path path_to_images, std::filesystem::path path_to_json, std::mutex& mutex, uint8_t& sync_var, std::condition_variable_any& cv): m_method(method), iID(iTotal), m_mutex(mutex), m_new_kp_rq(sync_var), m_cv(cv)
{
    iTotal++;
    iLiving++;
    LoadKeypointsFromImgs(path_to_images, path_to_json);
    for(size_t i = 0; i < m_images.size(); i++)
    {
        m_matchers.push_back(Matcher(m_method, (float)0.78));
    }

    for(size_t i = 0; i < m_images.size(); i++)
    {
        m_good_matches_filtered.push_back(std::vector<cv::DMatch>());
    }

}

AssemblyPart::~AssemblyPart()
{
    iLiving--;
}



void AssemblyPart::SetNewSceneParam(cv::Mat descriptor_scene, std::vector<cv::KeyPoint> keypoints_scene)
{
    m_descriptor_scene_local_cpy = descriptor_scene;
    m_keypoints_scene_local_cpy = keypoints_scene;
}


void AssemblyPart::FindMatches(const cv::Mat& descriptor_scene, const std::vector<cv::KeyPoint>& keypoints_scene, std::vector<std::vector<cv::Point>>& scene_corners)
{
    bool request_not_fullfiled = true;
    HANDLE hThread = GetCurrentThread();
    SetThreadPriority(hThread, THREAD_PRIORITY_TIME_CRITICAL);
    CloseHandle(hThread);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto start_time = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    {
        std::unique_lock<std::mutex> lk(m_mutex);
        m_new_kp_rq = true;
    }

    unsigned rect_cnt = 0;
    unsigned detected_cnt = 0;
    unsigned all_cnt = 0;

    while(true)
    {
        try
        {
            if(!m_descriptor_scene_local_cpy.empty() && !m_keypoints_scene_local_cpy.empty())
            {

                {
                    std::unique_lock<std::mutex> lk(m_mutex);
                    m_cv.wait(lk, [this] { return !m_new_kp_rq; });
                }
                start_time = std::chrono::high_resolution_clock::now();  
                all_cnt++;
                    
                #pragma omp parallel for
                for(int i = 0; i < m_matchers.size(); i++)
                {
                    try
                    {
                        m_matchers[i].Match(m_descriptors[i], m_descriptor_scene_local_cpy, m_keypoints_scene_local_cpy, m_good_matches_filtered[i]);
                    }
                    catch(cv::Exception& e)
                    {
                        std::cout << "Matcher exception: " << e.msg << std::endl;
                    }

                    std::vector<cv::Point2f> obj;
                    std::vector<cv::Point2f> scene;
                    std::vector<cv::Point2f> obj_corners(4);
                    if(m_good_matches_filtered[i].size() >= 10)
                    {
                        try
                        {
                            for(size_t j = 0; j < m_good_matches_filtered[i].size(); j++)
                            {
                                obj.push_back(m_keypoints[i][m_good_matches_filtered[i][j].queryIdx].pt);
                                scene.push_back(m_keypoints_scene_local_cpy[m_good_matches_filtered[i][j].trainIdx].pt);
                            }
                        }
                        catch(const std::out_of_range& e)
                        {
                            std::cerr << "Vector Out of Range Exception: " << e.what() << std::endl;
                        }
                        cv::Mat inliers;
                        cv::Mat H;
                        try
                        {
                            H = cv::findHomography(obj, scene, cv::RANSAC, 3, inliers);
                        }
                        catch(cv::Exception& e)
                        {
                            std::cout << "Homography exception: " << e.msg << std::endl;
                        }

                        /*
                        end_time = std::chrono::high_resolution_clock::now();
                        dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
                        std::cout << "Matching -> time elapsed:	" << dur << " ms" << std::endl;
                        */


                       
                            
                        if(PRINT == 1)
                        {
                            std::cout << "Assembly Part " << iID << " ";
                            std::cout << " [" << i << "] = " << "Keypoints == " << m_keypoints[i].size() << "   Good Matches == " << m_good_matches_filtered[i].size();
                        }
                            
                        double det = 0;
                        try
                        {
                            if(!H.empty())
                            {
                                det = H.at<double>(0, 0) * H.at<double>(1, 1) - H.at<double>(1, 0) * H.at<double>(0, 1);
                            }
                        }
                        catch(cv::Exception& e)
                        {
                            std::cout << "Homography determinant calculation exception: " << e.msg << std::endl;
                        }
                        bool acceptable_H = false;
                        try
                        {
                            if((det > 0.2) && (det < 2))
                            {
                                acceptable_H = true;
                                if(PRINT == 1)
                                {
                                    std::cout << "  ----> winner ";
                                }
                            }
                            else
                            {
                                if(!scene_corners[i].empty())
                                {
                                    std::fill(scene_corners[i].begin(), scene_corners[i].end(), cv::Point(0, 0));
                                }
                            }

                            obj_corners[0] = cv::Point2f(0, 0);
                            obj_corners[1] = cv::Point2f((float)m_images[i].cols, 0);
                            obj_corners[2] = cv::Point2f((float)m_images[i].cols, (float)m_images[i].rows);
                            obj_corners[3] = cv::Point2f(0, (float)m_images[i].rows);
                        }
                        catch(cv::Exception& e)
                        {
                            std::cout << "Homography calculation exception: " << e.msg << std::endl;
                        }

                        try
                        {
                            if(!H.empty() && acceptable_H)
                            {
                                std::vector<cv::Point2f> corners;
                                cv::perspectiveTransform(obj_corners, corners, H);

                                if(scene_corners[i].empty())
                                {
                                    for(size_t k = 0; k < 4; k++)
                                    {
                                        scene_corners[i].push_back(cv::Point(0, 0));
                                    }
                                }

                                bool isRect = true;
                                isRect = isRectangularShape(corners);
                                if(!isRect)
                                {
                                    if(!scene_corners[i].empty())
                                    {
                                        std::fill(scene_corners[i].begin(), scene_corners[i].end(), cv::Point(0, 0));
                                        rect_cnt++;
                                        continue;
                                    }
                                }
                                    
                                if(isRect)
                                {
                                    for(size_t j = 0; j < corners.size(); j++)
                                    {
                                        if((corners[j].x < 0) || (corners[j].y < 0))
                                        {
                                            if(!scene_corners[i].empty())
                                            {
                                                std::fill(scene_corners[i].begin(), scene_corners[i].end(), cv::Point(0, 0));
                                                break;
                                            }
                                        }
                                        scene_corners[i][j] = cv::Point((int)corners[j].x, (int)corners[j].y);
                                    }
                                }
                            }
                        }
                        catch(cv::Exception& e)
                        {
                            std::cout << "Perspective transform exception: " << e.msg << std::endl;
                        }
                    } // if enough good matches
                    else
                    {
                        if(!scene_corners[i].empty())
                        {
                            std::fill(scene_corners[i].begin(), scene_corners[i].end(), cv::Point(0, 0));
                        }
                    }
                }// for each matcher

                if(PRINT == 1)
                {
                    end_time = std::chrono::high_resolution_clock::now();
                    dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
                    std::cout << "__Matching -> time elapsed: " << dur << "ms" << std::endl;
                }

                {
                    std::unique_lock<std::mutex> lk(m_mutex);
                    m_new_kp_rq = true;
                }
            }
            else    //if copied KPs and Desc are empty - notify again
            {
                {
                    std::unique_lock<std::mutex> lk(m_mutex);
                    m_new_kp_rq = true;
                }
            }
            
        }
        catch(cv::Exception& e)
        {
            std::cout << "ALL OpenCV exception: " << e.msg << std::endl;
        }
        catch(const std::out_of_range& e)
        {
            std::cerr << "Vector Out of Range Exception: " << e.what() << std::endl;
        }
    }
}

void AssemblyPart::LoadKeypointsFromImgs(std::filesystem::path path_to_images, std::filesystem::path path_to_json)
{
    for(auto& p : std::filesystem::directory_iterator(path_to_json))
    {
        if(p.path().extension() == ".json")
        {
            if(p.path().string().find("descriptor") != std::string::npos)
            {
                cv::Mat descriptor_read;
                std::filesystem::path tmp = p.path();
                cv::String str(tmp.string());
                cv::FileStorage read_json(str, cv::FileStorage::READ);
                cv::String node_name = "desriptor_";

                for(size_t i = 0; i < 26; i++)       //max for each side
                {
                    {
                        cv::FileNode n2 = read_json[node_name + std::to_string(i)];
                        cv::read(n2, descriptor_read);
                        if(!descriptor_read.empty())
                        {
                            m_descriptors.push_back(descriptor_read);
                        }
                        else
                        {
                            m_descriptors.push_back(cv::Mat());
                        }
                    }
                }
                read_json.release();
            }

            if(p.path().string().find("keypoint") != std::string::npos)
            {
                std::vector<cv::KeyPoint> keypoints_read;
                std::filesystem::path tmp = p.path();
                cv::String str(tmp.string());
                cv::FileStorage read_json(str, cv::FileStorage::READ);
                cv::String node_name = "keypoints_";

                for(size_t i = 0; i < 26; i++)       //for each side
                {
                    {
                        cv::FileNode n2 = read_json[node_name + std::to_string(i)];
                        cv::read(n2, keypoints_read);
                        if(!keypoints_read.empty())
                        {
                            m_keypoints.push_back(keypoints_read);
                        }
                        else
                        {
                            m_keypoints.push_back(std::vector<cv::KeyPoint>());
                        }
                    }
                }
                read_json.release();
            }
        }
    }
    /********************************************************************************************************************/

    for(auto& p : std::filesystem::directory_iterator(path_to_images))
    {
        if(p.path().extension() == ".png")
        {
            std::filesystem::path tmp = p.path();
            cv::String str(tmp.string());
            m_images.push_back(cv::imread(str, cv::IMREAD_GRAYSCALE));
        }
    }

}
