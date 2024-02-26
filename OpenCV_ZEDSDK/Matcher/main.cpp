
#include <iostream>
#include <sl/Camera.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>


#define HOMOGRAPHY 1

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


void distanceFromCentroid(const std::vector<cv::Point2f>& points, cv::Point2f centroid, std::vector<double>& distances)
{
    std::vector<cv::Point2f>::const_iterator point;
    for(point = points.begin(); point != points.end(); ++point)
    {
        double distance = std::sqrt((point->x - centroid.x) * (point->x - centroid.x) + (point->y - centroid.y) * (point->y - centroid.y));
        distances.push_back(distance);
    }
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

    auto xd = zed.getCameraInformation();
}

int main()
{
    sl::Camera zed;
    InitCamera(zed);

    sl::Mat grabbed_frame;
    cv::Mat cv_frame;

    cv::Mat keypoints_frame;

    /********************************************************************************************************************/
    cv::Mat keypoints_frame_next;
    /********************************************************************************************************************/

    cv::Ptr<cv::BRISK> detector_brisk = cv::BRISK::create();
    //cv::Ptr<cv::ORB> detector_orb = cv::ORB::create();
    //cv::Ptr<cv::SIFT> detector_sift = cv::SIFT::create();
    std::vector<cv::KeyPoint> keypoints_scene;
    cv::Mat descriptor;

    auto start_time = std::chrono::high_resolution_clock::now();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();


    auto start_time_matcher = std::chrono::high_resolution_clock::now();
    auto end_time_matcher = std::chrono::high_resolution_clock::now();
    auto dur_matcher = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    /*********************************  Reading the descriptor    *******************************************************/

    const unsigned max_desc_count = 1;
    std::vector<cv::Mat> descriptors_from_file;
    cv::Mat descriptor_read;
    cv::FileStorage store("descriptor_Raff_1.json", cv::FileStorage::READ);
    cv::String node_name = "desriptor_";

    for(size_t i = 0; i < max_desc_count; i++)
    {
        cv::String tmp = node_name + std::to_string(i);
        cv::FileNode n2 = store[tmp];
        cv::read(n2, descriptor_read);
        descriptors_from_file.push_back(descriptor_read);
    }
    store.release();
    /********************************************************************************************************************/



    /********************************************************************************************************************/
    std::vector<cv::Mat> descriptors_from_file_next;
    cv::Mat descriptor_read_next;
    cv::FileStorage store_next("descriptor_Raff_2.json", cv::FileStorage::READ);
    //cv::String node_name = "desriptor_";

  
    for(size_t i = 0; i < max_desc_count; i++)
    {
        cv::String tmp = node_name + std::to_string(i);
        cv::FileNode n2 = store_next[tmp];
        cv::read(n2, descriptor_read_next);
        descriptors_from_file_next.push_back(descriptor_read_next);
    }
    store_next.release();
    /********************************************************************************************************************/


    //cv::Mat img_read = cv::imread("image0.jpg", cv::IMREAD_GRAYSCALE);
    cv::Mat img_read = cv::imread("3D_Raff_front1.png", cv::IMREAD_GRAYSCALE);
    cv::Mat img_read_keypoints = img_read.clone();

    cv::Mat img_read_homography = img_read.clone();
    std::vector<cv::KeyPoint> keypoints_object;
    cv::Mat descriptor_original;
    cv::Mat desc_dummy;



    /********************************************************************************************************************/
    cv::Mat img_read_next = cv::imread("3D_Raff_side1.png", cv::IMREAD_GRAYSCALE);
    cv::Mat img_read_keypoints_next = img_read_next.clone();
    std::vector<cv::KeyPoint> keypoints_object_next;
    /********************************************************************************************************************/


    try
    {
        detector_brisk->detectAndCompute(img_read, cv::noArray(), keypoints_object, desc_dummy, false);       //just for demonstration purposes

        /********************************************************************************************************************/
        detector_brisk->detectAndCompute(img_read_next, cv::noArray(), keypoints_object_next, desc_dummy, false);       //just for demonstration purposes
        /********************************************************************************************************************/


    }
    catch(const cv::Exception& e)
    {
        std::cerr << "OpenCV exception: " << e.what() << std::endl;
        return -1;
    }

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

    /********************************************************************************************************************/
    cv::Ptr<cv::DescriptorMatcher> matcher_next = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
    /********************************************************************************************************************/

    cv::Ptr<cv::BFMatcher> matcher_bf = cv::BFMatcher::create(cv::BFMatcher::BRUTEFORCE_HAMMING, true);
    cv::Ptr<cv::DescriptorMatcher> matcher_flann = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<cv::DMatch> > knn_matches;
    const float ratio_thresh = 0.8f;        //poznamka v papieri -> napisat do DP [Distinctive Image Features from Scale-Invariant Keypoints]   !!!!!!!!!!!!!

    while(true)
    {
        auto returned_state = zed.grab();
        if(returned_state == sl::ERROR_CODE::SUCCESS)
        {
            zed.retrieveImage(grabbed_frame, sl::VIEW::LEFT, sl::MEM::CPU);
            cv_frame = slMat2cvMat(grabbed_frame);
        }

        cv::imshow("Camera feed", cv_frame);        

        std::cout << "Snapshot taken [BRISK]: " << std::endl;
        //detector, descriptor creation
        //cv_frame.copyTo(keypoints_frame);
        cv::cvtColor(cv_frame, keypoints_frame, cv::COLOR_BGR2GRAY);
        /********************************************************************************************************************/

        cv::cvtColor(cv_frame, keypoints_frame_next, cv::COLOR_BGR2GRAY);

        /********************************************************************************************************************/

        start_time = std::chrono::high_resolution_clock::now();

        detector_brisk->detectAndCompute(keypoints_frame, cv::noArray(), keypoints_scene, descriptor, false);

         /********************************************************************************************************************/
        std::vector<cv::DMatch> good_matches_next;
        knn_matches.clear();
        //matcher->clear();
        if((!descriptor.empty() && !descriptors_from_file_next[0].empty()))
        {
            matcher_next->knnMatch(descriptors_from_file_next[0], descriptor, knn_matches, 2);
            //matcher_bf->match(descriptor, descriptors_from_file[0], good_matches, cv::noArray());
            //matcher_flann->knnMatch(descriptor, descriptors_from_file[0], knn_matches, 2);
            for(size_t i = 0; i < knn_matches.size(); i++)
            {
                if(knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
                {
                    good_matches_next.push_back(knn_matches[i][0]);
                }
            }
        }
        /********************************************************************************************************************/

        end_time = std::chrono::high_resolution_clock::now();
        dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        std::cout << dur << " ms" << std::endl;
        std::cout << "Camera Frame kp: " << keypoints_scene.size() << std::endl;
        std::cout << "Original Frame kp: " << keypoints_object.size() << std::endl;
        //std::cout << "Good Matches: " << good_matches.size() << std::endl;
        std::cout << "Good Matches: " << good_matches_next.size() << std::endl;
        std::cout << "-----------------------------" << std::endl;

        start_time_matcher = std::chrono::high_resolution_clock::now();

        /*********************************  Filter matches using the Lowe's ratio test    *******************************************************/
        
        std::vector<cv::DMatch> good_matches;
        if((!descriptor.empty() && !descriptors_from_file[0].empty()))
        {
            matcher->knnMatch(descriptors_from_file[0], descriptor, knn_matches, 2);
            //matcher_bf->match(descriptor, descriptors_from_file[0], good_matches, cv::noArray());
            //matcher_flann->knnMatch(descriptor, descriptors_from_file[0], knn_matches, 2);
            for(size_t i = 0; i < knn_matches.size(); i++)
            {
                if(knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
                {
                   good_matches.push_back(knn_matches[i][0]);
                }
            }
        }

        /********************************************************************************************************************/
            
        //just for demonstration purposes
        cv::Mat img_matches;

        /********************************************************************************************************************/
        cv::Mat img_matches_next;
        /********************************************************************************************************************/
        if(!good_matches.empty() && !good_matches_next.empty())
        {
            try
            {
                std::vector<cv::Point2f> points;
                std::vector<cv::KeyPoint>::iterator keypoint;

                /************************************* Centroid calculation *********************************************************/
                for(size_t i = 0; i < good_matches.size(); i++)
                {
                    points.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
                }
                cv::Moments m = cv::moments(points, false);
                cv::Point2f centroid(m.m10/m.m00, m.m01 / m.m00);
                cv::circle(keypoints_frame, centroid, 50, cv::Scalar(0, 0, 255), 5);
                /********************************************************************************************************************/

                /************************************* Spatial filtering *********************************************************/

                std::vector<double> distances;
                distanceFromCentroid(points, centroid, distances);

                cv::Scalar mu, sigma;
                cv::meanStdDev(distances, mu, sigma);

                //std::cout << mu.val[0] << ", " << sigma.val[0] << std::endl;

                std::vector<cv::KeyPoint> filtered;
                std::vector<double>::iterator distance;
                for(size_t i = 0; i < distances.size(); ++i)
                {
                    if(distances[i] < (mu.val[0] + 2.0 * sigma.val[0]))
                    {
                        filtered.push_back(keypoints_scene[good_matches[i].trainIdx]);
                    }
                }


                /********************************************************************************************************************/

                end_time = std::chrono::high_resolution_clock::now();
                dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

                std::cout << "Matcher time elapsed:  " << dur << " ms" << std::endl;
                std::cout << "-----------------------------" << std::endl;


                std::vector<cv::Point2f> points_next;
                //std::vector<cv::KeyPoint>::iterator keypoint;
                for(size_t i = 0; i < good_matches_next.size(); i++)
                {
                    points_next.push_back(keypoints_scene[good_matches_next[i].trainIdx].pt);
                }
                cv::Moments m_next = cv::moments(points_next, false);
                cv::Point2f centroid_next(m_next.m10 / m_next.m00, m_next.m01 / m_next.m00);
                cv::circle(keypoints_frame_next, centroid_next, 50, cv::Scalar(0, 0, 255), 5);


                std::vector<double> distances_next;
                distanceFromCentroid(points_next, centroid_next, distances_next);

                cv::Scalar mu_next, sigma_next;
                cv::meanStdDev(distances_next, mu_next, sigma_next);

                //std::cout << mu.val[0] << ", " << sigma.val[0] << std::endl;

                std::vector<cv::KeyPoint> filtered_next;
                std::vector<double>::iterator distance_next;
                for(size_t i = 0; i < distances_next.size(); ++i)
                {
                    if(distances_next[i] < (mu_next.val[0] + 2.0 * sigma_next.val[0]))
                    {
                        filtered_next.push_back(keypoints_scene[good_matches_next[i].trainIdx]);
                    }
                }
                /********************************************************************************************************************/

                drawMatches(img_read, keypoints_object, keypoints_frame, keypoints_scene, good_matches, img_matches, cv::Scalar::all(-1),
                            cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                /*
                drawMatches(img_read_next, keypoints_object_next, keypoints_frame_next, keypoints_scene, good_matches_next, img_matches_next, cv::Scalar::all(-1),
                            cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                */

                cv::drawKeypoints(keypoints_frame, filtered, keypoints_frame);
                imshow("Keypoints CAMERA", keypoints_frame);
                //cv::drawKeypoints(img_read_keypoints, keypoints_object, img_read_keypoints);
                /********************************************************************************************************************/
                cv::drawKeypoints(keypoints_frame_next, filtered_next, keypoints_frame_next);
                imshow("Keypoints NEXT CAMERA", keypoints_frame_next);
                /********************************************************************************************************************/
            }
            catch(const cv::Exception& e) 
            {
                std::cerr << "OpenCV exception: " << e.what() << std::endl;
                std::cerr << "Camera Frame kp: " << keypoints_scene.size() << std::endl;
                std::cerr << "Original Frame kp: " << keypoints_object.size() << std::endl;
                std::cout << "Good Matches: " << good_matches.size() << std::endl;
                return -1;
            }

            //-- Show detected matches
            if(!img_matches.empty())
            {
                imshow("Good Matches", img_matches);
            }

            /********************************************************************************************************************/
            if(!img_matches_next.empty())
            {
                imshow("Good Matches Next", img_matches_next);
            }
            /********************************************************************************************************************/
            //imshow("Keypoints CAMERA", keypoints_frame);
            //imshow("Keypoints IMAGE", img_read_keypoints);
            cv::Rect myROI(500, 350, 125, 125);
            cv::Mat test = img_read(myROI);
            
            /*********************************  Find homography    *******************************************************/
#ifdef HOMOGRAPHY

            /*
            * TU SA NEPOUZIVAJU GOOD MATCHES ODFILTROVANE CEZ PRIESTOROVU LOKALITU
            */
            
            start_time = std::chrono::high_resolution_clock::now();

            std::vector<cv::Point2f> obj;
            std::vector<cv::Point2f> scene;

            std::vector<cv::Point2f> obj_corners(4);
            std::vector<cv::Point2f> scene_corners(4);
            try
            {
                if(good_matches.size() >= 4)
                {
                    for(size_t i = 0; i < good_matches.size(); i++)
                    {
                        obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
                        scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
                    }

                    cv::Mat H = cv::findHomography(obj, scene, cv::RANSAC, 9);
                    //-- Get the corners from the image_1 ( the object to be "detected" )
                    cv::Rect myROI(500, 350, 125, 125);
                    cv::Mat cut_out_img = img_read_homography(myROI);
                    cut_out_img = img_read_homography;

                    obj_corners[0] = cv::Point2f(0, 0);
                    obj_corners[1] = cv::Point2f((float)cut_out_img.cols, 0);
                    obj_corners[2] = cv::Point2f((float)cut_out_img.cols, (float)cut_out_img.rows);
                    obj_corners[3] = cv::Point2f(0, (float)cut_out_img.rows);
                    //obj_corners[0] = cv::Point2f(500, 350);
                    //obj_corners[1] = cv::Point2f(625, 475);
                    //obj_corners[2] = cv::Point2f(625, 475);
                    //obj_corners[3] = cv::Point2f(500, 350);
                    if(!H.empty())
                    {
                        cv::perspectiveTransform(obj_corners, scene_corners, H);

                        double fx = 730;
                        double fy = 730;

                        // Assuming the image resolution is 640x480 (you need to adjust this based on your actual image size)
                        double cx = 635;  // Optical center X-coordinate in pixels
                        double cy = 361;  // Optical center Y-coordinate in pixels

                        // Create the camera matrix
                        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

                        std::vector<cv::Mat> rotations, translations, normals;
                        cv::decomposeHomographyMat(H, cameraMatrix, rotations, translations, normals);

                        // Print the rotation and translation
                       
                        cv::Vec3d rvec;
                        cv::Rodrigues(rotations[0], rvec);

                        // Convert rotation vector to Euler angles (using RPY convention)
                        cv::Vec3d euler_angles = rvec * (180.0 / CV_PI);  // Convert radians to degrees

                        // Print Euler angles
                        std::cout << "Roll (X-axis): " << euler_angles[0] << " degrees" << std::endl;
                        std::cout << "Pitch (Y-axis): " << euler_angles[1] << " degrees" << std::endl;
                        std::cout << "Yaw (Z-axis): " << euler_angles[2] << " degrees" << std::endl;

                    }

                    bool flag = false;
                    for(unsigned i = 0; i < scene_corners.size(); i++)
                    {
                        if(scene_corners[i].x > 1080 || scene_corners[i].x < 0)
                        {
                            flag = true;
                        }
                        if(scene_corners[i].y > 720 || scene_corners[i].y < 0)
                        {
                            flag = true;
                        }
                    }

                    end_time = std::chrono::high_resolution_clock::now();
                    dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
                    std::cout << "Homography" << dur << " ms" << std::endl;
                    std::cout << "---------------------------" << std::endl;

                    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
                    if(!flag)
                    {
                        
                        line(img_matches, scene_corners[0] + cv::Point2f((float)cut_out_img.cols, 0),
                             scene_corners[1] + cv::Point2f((float)cut_out_img.cols, 0), cv::Scalar(0, 255, 0), 4);
                        line(img_matches, scene_corners[1] + cv::Point2f((float)cut_out_img.cols, 0),
                             scene_corners[2] + cv::Point2f((float)cut_out_img.cols, 0), cv::Scalar(0, 255, 0), 4);
                        line(img_matches, scene_corners[2] + cv::Point2f((float)cut_out_img.cols, 0),
                             scene_corners[3] + cv::Point2f((float)cut_out_img.cols, 0), cv::Scalar(0, 255, 0), 4);
                        line(img_matches, scene_corners[3] + cv::Point2f((float)cut_out_img.cols, 0),
                             scene_corners[0] + cv::Point2f((float)cut_out_img.cols, 0), cv::Scalar(0, 255, 0), 4);
                                            
                        
                        /*
                        line(img_matches, scene_corners[0] + cv::Point2f((float)img_read_homography.cols, 0),
                             scene_corners[1] + cv::Point2f((float)img_read_homography.cols, 0), cv::Scalar(0, 255, 0), 4);
                        line(img_matches, scene_corners[1] + cv::Point2f((float)img_read_homography.cols, 0),
                             scene_corners[2] + cv::Point2f((float)img_read_homography.cols, 0), cv::Scalar(0, 255, 0), 4);
                        line(img_matches, scene_corners[2] + cv::Point2f((float)img_read_homography.cols, 0),
                             scene_corners[3] + cv::Point2f((float)img_read_homography.cols, 0), cv::Scalar(0, 255, 0), 4);
                        line(img_matches, scene_corners[3] + cv::Point2f((float)img_read_homography.cols, 0),
                             scene_corners[0] + cv::Point2f((float)img_read_homography.cols, 0), cv::Scalar(0, 255, 0), 4);
                        */
                        /*
                        cv::line(img_matches, scene_corners[0], scene_corners[1], cv::Scalar(0, 255, 0), 2);
                        cv::line(img_matches, scene_corners[1], scene_corners[2], cv::Scalar(0, 255, 0), 2);
                        cv::line(img_matches, scene_corners[2], scene_corners[3], cv::Scalar(0, 255, 0), 2);
                        cv::line(img_matches, scene_corners[3], scene_corners[0], cv::Scalar(0, 255, 0), 2);
                        */
                    }
                    flag = false;
                    //-- Show detected matches
                }
            }
            catch(const cv::Exception& e)
            {
                std::cerr << "OpenCV exception: " << e.what() << std::endl;
                return -1;
            }
            imshow("Good Matches & Object detection", img_matches);


            obj.clear();
            scene.clear();
#endif
            /********************************************************************************************************************/
            

            /******************** clear ***********************/
            keypoints_scene.clear();
            //descriptor.release();
            good_matches.clear();
            knn_matches.clear();

            good_matches_next.clear();
            knn_matches.clear();
            
        }
            cv::waitKey(50);  //
    }
}
