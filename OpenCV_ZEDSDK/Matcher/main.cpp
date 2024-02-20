
#include <iostream>
#include <sl/Camera.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>


//#define HOMOGRAPHY 1

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




int main()
{
    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_parameters.camera_fps = 30;

    sl::Camera zed;
    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if(returned_state != sl::ERROR_CODE::SUCCESS) {
        std::cout << "Camera error: " << returned_state << std::endl;
    }

    sl::Mat grabbed_frame;
    cv::Mat cv_frame;

    cv::Mat keypoints_frame;

    cv::Ptr<cv::BRISK> detector_brisk = cv::BRISK::create();
    //cv::Ptr<cv::ORB> detector_orb = cv::ORB::create();
    //cv::Ptr<cv::SIFT> detector_sift = cv::SIFT::create();
    std::vector<cv::KeyPoint> keypoints_scene;
    cv::Mat descriptor;

    auto start_time = std::chrono::high_resolution_clock::now();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    /*********************************  Reading the descriptor    *******************************************************/

    const unsigned max_desc_count = 1;
    std::vector<cv::Mat> descriptors_from_file;
    cv::Mat descriptor_read;
    cv::FileStorage store("descriptor_BRISK_Eiffel.json", cv::FileStorage::READ);
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
    //cv::Mat img_read = cv::imread("image0.jpg", cv::IMREAD_GRAYSCALE);
    cv::Mat img_read = cv::imread("image0_test.png", cv::IMREAD_GRAYSCALE);
    cv::Mat img_read_keypoints = img_read.clone();
    cv::Mat img_read_homography = img_read.clone();
    std::vector<cv::KeyPoint> keypoints_object;
    cv::Mat descriptor_original;
    cv::Mat desc_dummy;

    try
    {
        detector_brisk->detectAndCompute(img_read, cv::noArray(), keypoints_object, desc_dummy, false);       //just for demonstration purposes
    }
    catch(const cv::Exception& e)
    {
        std::cerr << "OpenCV exception: " << e.what() << std::endl;
        return -1;
    }

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
    cv::Ptr<cv::BFMatcher> matcher_bf = cv::BFMatcher::create(cv::BFMatcher::BRUTEFORCE_HAMMING, true);
    cv::Ptr<cv::DescriptorMatcher> matcher_flann = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<cv::DMatch> > knn_matches;
    const float ratio_thresh = 0.8f;        //poznamka v papieri -> napisat do DP [Distinctive Image Features from Scale-Invariant Keypoints]   !!!!!!!!!!!!!

    while(true)
    {
        returned_state = zed.grab();
        if(returned_state == sl::ERROR_CODE::SUCCESS)
        {
            zed.retrieveImage(grabbed_frame, sl::VIEW::LEFT, sl::MEM::CPU);
            cv_frame = slMat2cvMat(grabbed_frame);
        }

        cv::imshow("Camera feed", cv_frame);        

        std::cout << "Snapshot taken [BRISK]: ";
        //detector, descriptor creation
        //cv_frame.copyTo(keypoints_frame);
        cv::cvtColor(cv_frame, keypoints_frame, cv::COLOR_BGR2GRAY);
        start_time = std::chrono::high_resolution_clock::now();

        detector_brisk->detectAndCompute(keypoints_frame, cv::noArray(), keypoints_scene, descriptor, false);

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
            
        end_time = std::chrono::high_resolution_clock::now();
        dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            
        std::cout << dur << " ms" << std::endl;
        std::cout << "Camera Frame kp: " << keypoints_scene.size() << std::endl;
        std::cout << "Original Frame kp: " << keypoints_object.size() << std::endl;
        std::cout << "Good Matches: " << good_matches.size() << std::endl;
        std::cout << "-----------------------------" << std::endl;

        //just for demonstration purposes
        cv::Mat img_matches;
        if(!good_matches.empty())
        {
            try
            {

                /*drawMatches(keypoints_frame, keypoints_scene, img_read, keypoints_object, good_matches, img_matches, cv::Scalar::all(-1),
                            cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT);*/


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


                drawMatches(img_read, keypoints_object, keypoints_frame, keypoints_scene, good_matches, img_matches, cv::Scalar::all(-1),
                            cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT);

                cv::drawKeypoints(keypoints_frame, filtered, keypoints_frame);
                imshow("Keypoints CAMERA", keypoints_frame);
                //cv::drawKeypoints(img_read_keypoints, keypoints_object, img_read_keypoints);
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
            //imshow("Keypoints CAMERA", keypoints_frame);
            //imshow("Keypoints IMAGE", img_read_keypoints);
            cv::Rect myROI(500, 350, 125, 125);
            cv::Mat test = img_read(myROI);
            
            /*********************************  Find homography    *******************************************************/
#ifdef HOMOGRAPHY

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
                    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
                    if(!flag)
                    {
                        /*
                        line(img_matches, scene_corners[0] + cv::Point2f((float)cut_out_img.cols, 0),
                             scene_corners[1] + cv::Point2f((float)cut_out_img.cols, 0), cv::Scalar(0, 255, 0), 4);
                        line(img_matches, scene_corners[1] + cv::Point2f((float)cut_out_img.cols, 0),
                             scene_corners[2] + cv::Point2f((float)cut_out_img.cols, 0), cv::Scalar(0, 255, 0), 4);
                        line(img_matches, scene_corners[2] + cv::Point2f((float)cut_out_img.cols, 0),
                             scene_corners[3] + cv::Point2f((float)cut_out_img.cols, 0), cv::Scalar(0, 255, 0), 4);
                        line(img_matches, scene_corners[3] + cv::Point2f((float)cut_out_img.cols, 0),
                             scene_corners[0] + cv::Point2f((float)cut_out_img.cols, 0), cv::Scalar(0, 255, 0), 4);
                        */                    

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

            
        }
            cv::waitKey(50);  //
    }
}
