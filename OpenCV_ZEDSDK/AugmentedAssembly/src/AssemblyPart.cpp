#include "AssemblyPart.h"


size_t AssemblyPart::iLiving = 0;
size_t AssemblyPart::iTotal = 0;


AssemblyPart::AssemblyPart(Method method, std::filesystem::path path_to_images, std::filesystem::path path_to_json, std::mutex& mutex, std::atomic<bool>& sync_var): m_method(method), iID(iTotal), m_mutex(mutex), m_new_kp_rq(sync_var)
{
    iTotal++;
    iLiving++;
	GetKeypointsFromImgs(path_to_images, path_to_json);
    for(size_t i = 0; i < m_descriptors.size(); i++)
    {
        m_matchers.push_back(Matcher(m_method, 0.85));
    }
    
    for(size_t i = 0; i < m_descriptors.size(); i++)
    {
        m_good_matches_filtered.push_back(std::vector<cv::DMatch>());
    }

}

AssemblyPart::~AssemblyPart()
{
    iLiving--;
}

void AssemblyPart::SetDescriptors(std::vector<cv::Mat>& desc)
{
	m_descriptors = desc;
}

std::vector<cv::Mat> AssemblyPart::GetDescriptors()
{
	return m_descriptors;
}

std::vector<cv::Mat> AssemblyPart::GetImages()
{
    return m_images;
}

std::vector<std::vector<cv::KeyPoint>> AssemblyPart::GetKeypoints()
{
    return m_keypoints;
}

std::vector<std::vector<cv::DMatch>> AssemblyPart::GetFilteredMatches()
{
    return m_good_matches_filtered;
}

std::vector<cv::KeyPoint> AssemblyPart::GetKpSceneCopy()
{
    return m_keypoints_scene_local_cpy;
}



void AssemblyPart::SetNewSceneParam(cv::Mat descriptor_scene, std::vector<cv::KeyPoint> keypoints_scene)
{
    m_descriptor_scene_local_cpy = descriptor_scene;
    m_keypoints_scene_local_cpy = keypoints_scene;
}


void AssemblyPart::FindMatches(const cv::Mat& descriptor_scene, const std::vector<cv::KeyPoint>& keypoints_scene, std::vector<cv::Point2f>& scene_corners)
{
    bool request_not_fullfiled = true;
    cv::Ptr<cv::DescriptorMatcher> m_matcher_brisk = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

    /*
    HANDLE hThread = GetCurrentThread();
	SetThreadAffinityMask(hThread, iID + 4);
	SetThreadPriority(hThread, THREAD_PRIORITY_HIGHEST);
	CloseHandle(hThread);
    */
    while(true)
    {
        //std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if(!m_descriptor_scene_local_cpy.empty() && !m_keypoints_scene_local_cpy.empty())
        {
            m_mutex.lock();
            request_not_fullfiled = m_new_kp_rq.load(std::memory_order_acquire);
            m_mutex.unlock();
            if(!request_not_fullfiled)
            {
                /*
                * do the matching
                */
                {
                    for(size_t i = 0; i < m_matchers.size(); i++)
                    {
                        std::vector< std::vector<cv::DMatch>> knn_matches;
                        m_matchers[i].Match(m_descriptors[i], m_descriptor_scene_local_cpy, m_keypoints_scene_local_cpy, m_good_matches_filtered[i]);
                        knn_matches.clear();
                    }
                    //std::cout << "Assembly Part local " << iID << "\t" << m_descriptor_scene_local_cpy.size() << "\tAssembly Part filtered" << iID << "\t" << m_good_matches_filtered[0].size() << std::endl;

                    std::vector<cv::Point2f> obj;
                    std::vector<cv::Point2f> scene;

                    std::vector<cv::Point2f> obj_corners(4);
                    //std::vector<cv::Point2f> scene_corners(4);
                    try
                    {
                        if(m_good_matches_filtered[0].size() >= 4)
                        {
                            
                            for(size_t i = 0; i < m_good_matches_filtered[0].size(); i++)
                            {
                                obj.push_back(m_keypoints[0][m_good_matches_filtered[0][i].queryIdx].pt);
                                scene.push_back(m_keypoints_scene_local_cpy[m_good_matches_filtered[0][i].trainIdx].pt);
                            }          
                            cv::Mat H = cv::findHomography(obj, scene, cv::RANSAC);
                            
                            obj_corners[0] = cv::Point2f(0, 0);
                            obj_corners[1] = cv::Point2f((float)m_images[0].cols, 0);
                            obj_corners[2] = cv::Point2f((float)m_images[0].cols, (float)m_images[0].rows);
                            obj_corners[3] = cv::Point2f(0, (float)m_images[0].rows);

                            if(!H.empty())
                            {
                                cv::perspectiveTransform(obj_corners, scene_corners, H);

                                double fx = 730;
                                double fy = 730;
                                double cx = 635;  // Optical center X-coordinate in pixels
                                double cy = 361;  // Optical center Y-coordinate in pixels

                                cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

                                std::vector<cv::Mat> rotations, translations, normals;
                                cv::decomposeHomographyMat(H, cameraMatrix, rotations, translations, normals);

                                cv::Vec3d rvec;
                                cv::Rodrigues(rotations[0], rvec);

                                cv::Vec3d euler_angles = rvec * (180.0 / CV_PI);  // Convert radians to degrees

                                
                                std::cout << "Roll (X-axis): " << euler_angles[0] << " degrees" << std::endl;
                                std::cout << "Pitch (Y-axis): " << euler_angles[1] << " degrees" << std::endl;
                                std::cout << "Yaw (Z-axis): " << euler_angles[2] << " degrees" << std::endl;
                                std::cout << "Matches: " << m_good_matches_filtered[0].size() << std::endl;
                                std::cout << "-------------------------------------------------------------" << std::endl;
                                
                            }
                        }
                    }
                    catch(const cv::Exception& e)
                    {
                        std::cerr << "OpenCV exception: " << e.what() << std::endl;
                    }

                    m_mutex.lock();
                    m_new_kp_rq.store(true, std::memory_order_release);	//notify AugmentedAssembly object about a new request
                    m_mutex.unlock();
                }
            }
        }
        else
        {
            m_mutex.lock();
            m_new_kp_rq.store(true, std::memory_order_release);	//notify AugmentedAssembly object about a new request
            m_mutex.unlock();
        }

    }




	//FindBestMatch();
}

void AssemblyPart::GetKeypointsFromImgs(std::filesystem::path path_to_images, std::filesystem::path path_to_json)
{
    for(auto& p : std::filesystem::directory_iterator(path_to_json))
    {
        //std::cout << p.path() << '\n';
        if(p.path().extension() == ".json")
        {
            if(p.path().string().find("descriptor") != std::string::npos)
            {
                cv::Mat descriptor_read;
                std::filesystem::path tmp = p.path();
                cv::String str(tmp.string());
                cv::FileStorage read_json(str, cv::FileStorage::READ);
                cv::String node_name = "desriptor_";

                for(size_t i = 0; i < 6; i++)       //for each side
                {
                    cv::FileNode n2 = read_json[node_name + std::to_string(i)];
                    cv::read(n2, descriptor_read);
                    if(!descriptor_read.empty())
                    {
                        m_descriptors.push_back(descriptor_read);
                    }
                }
                read_json.release();
            }

            if(p.path().string().find("keypoint") != std::string::npos)
            {
                //cv::Mat keypoints_read;
                std::vector<cv::KeyPoint> keypoints_read;
                std::filesystem::path tmp = p.path();
                cv::String str(tmp.string());
                cv::FileStorage read_json(str, cv::FileStorage::READ);
                cv::String node_name = "keypoints_";

                for(size_t i = 0; i < 6; i++)       //for each side
                {
                    cv::FileNode n2 = read_json[node_name + std::to_string(i)];
                    cv::read(n2, keypoints_read);
                    if(!keypoints_read.empty())
                    {
                        m_keypoints.push_back(keypoints_read);
                    }
                }
                read_json.release();
            }
        }

        
        /********************************************************************************************************************/

        for(auto& p : std::filesystem::directory_iterator(path_to_images))
        {
            //std::cout << p.path() << '\n';
            if(p.path().extension() == ".png")
            for(size_t i = 0; i < 6; i++)       //for each side
            {
                std::filesystem::path tmp = p.path();
                cv::String str(tmp.string());
                m_images.push_back(cv::imread(str, cv::IMREAD_GRAYSCALE));
            }
        }
    }

}
