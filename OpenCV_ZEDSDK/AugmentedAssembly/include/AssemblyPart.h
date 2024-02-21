#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

class AssemblyPart {
public:
    AssemblyPart();                 // Default constructor
    ~AssemblyPart();                // Destructor

    void SetDescriptor(cv::Mat& desc);
    cv::Mat GetDescriptor();

private:
    std::vector<cv::Mat> m_images;
    std::vector<std::vector<cv::KeyPoint>> m_keypoints;       //keypoints per image
    cv::Mat m_descriptor;
    
};


/*  GOING OVER ALL THE REFERENCE IMAGESE
for i, reference_image in enumerate([img_reference1, img_reference2]):
    # Detect keypoints and extract descriptors for the reference image
    keypoints_reference, descriptors_reference = detector.detectAndCompute(reference_image, None)

    # Match descriptors using KNN
    matches = bf.knnMatch(descriptors_query, descriptors_reference, k=2)

    # Apply ratio test
    good_matches = []
    for m, n in matches:
        if m.distance < 0.75 * n.distance:
            good_matches.append(m)

    # Print the number of good matches for each reference image
    print(f"Number of good matches for reference image {i+1}: {len(good_matches)}")

    # Optionally, you can store or analyze the good matches further
    # For example, you might want to compute an overall score based on the number of good matches

    # Determine the best response based on your specific criteria
    threshold = 10  # Adjust this threshold based on your application
    if len(good_matches) > threshold:
        print(f"Query image is similar to reference image {i+1}")
    else:
        print(f"Query image is not similar to reference image {i+1}")

*/