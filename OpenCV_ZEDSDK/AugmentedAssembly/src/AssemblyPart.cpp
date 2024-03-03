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
        m_matchers.push_back(Matcher(m_method, 0.8));
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

void AssemblyPart::SetNewSceneParam(cv::Mat& descriptor_scene, std::vector<cv::KeyPoint> keypoints_scene)
{
    m_descriptor_scene_local_cpy = descriptor_scene;
    m_keypoints_scene_local_cpy = keypoints_scene;
}


void AssemblyPart::FindMatches(const cv::Mat& descriptor_scene, const std::vector<cv::KeyPoint>& keypoints_scene)
{
    bool request_not_fullfiled = true;
    cv::Ptr<cv::DescriptorMatcher> m_matcher_brisk = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

    while(true)
    {
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
                    //std::this_thread::sleep_for(std::chrono::milliseconds(iID * 1000 + 500));
                    std::cout << "Assembly Part local " << iID << "\t" << m_descriptor_scene_local_cpy.size() << std::endl;
                    std::cout << "Assembly Part filtered" << iID << "\t" << m_good_matches_filtered[0].size() << std::endl;
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
