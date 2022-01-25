#include <numeric>

#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource,
                      cv::Mat &descRef, std::vector<cv::DMatch> &matches, std::string descriptorType,
                      std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F)
        {  // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV
           // implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }

        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }
    else
    {
        string error_message = "Unsupported matcher type: " + matcherType;
        throw runtime_error(error_message);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    {
        // nearest neighbor (best match)
        matcher->match(descSource, descRef, matches);  // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    {  // k nearest neighbors (k=2)
        vector<vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, 2);

        // filter ambiguos matches using descriptor distance ratio test
        double minDescDistRatio = 0.8;
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {
            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
            {
                matches.push_back((*it)[0]);
            }
        }
    }
    else
    {
        string error_message = "Unsupported match selector type: " + selectorType;
        throw runtime_error(error_message);
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {
        int threshold = 30;  // FAST/AGAST detection threshold score.
        int octaves = 3;     // detection octaves (use 0 to do single scale)
        float patternScale =
            1.0f;  // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(64);
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        extractor = cv::SIFT::create();
    }
    else
    {
        string error_message = "Unsupported keypoint descriptor type: " + descriptorType;
        throw runtime_error(error_message);
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "   " << descriptorType << " descriptor extraction: descriptors matrix zise: " << descriptors.size()
         << ", duration: " << 1000 * t / 1.0 << " ms" << endl;
}

void visualizeKeypoints(const std::vector<cv::KeyPoint> &keypoints, const cv::Mat &img, std::string detectorType)
{
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = detectorType + " Corner Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cv::waitKey(0);
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize =
        4;  //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0;  // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance);  // max. num. of keypoints

    double qualityLevel = 0.01;  // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {
        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "   Shi-Tomasi keypoints detection: " << keypoints.size() << " keypoints, duration: " << 1000 * t / 1.0
         << " ms" << endl;

    // visualize results
    if (bVis)
    {
        visualizeKeypoints(keypoints, img, "Shi-Tomasi");
    }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // Detector parameters
    int blockSize = 2;        // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;     // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100;    // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;          // Harris parameter (see equation for details)
    double maxOverlap = 0.0;  // max. permissible overlap between two features in %, used during non-maxima suppression

    double t = (double)cv::getTickCount();

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    for (size_t j = 0; j < dst_norm.rows; j++)
    {
        for (size_t i = 0; i < dst_norm.cols; i++)
        {
            int response = (int)dst_norm.at<float>(j, i);
            if (response > minResponse)  // only store points above a threshold
            {
                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response;

                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                bool overlaped = false;
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                    if (kptOverlap > maxOverlap)
                    {
                        overlaped = true;
                        if (newKeyPoint.response > (*it).response)
                        {                       // if overlap is >t AND response is higher for new kpt
                            *it = newKeyPoint;  // replace old key point with new one
                            break;
                        }
                    }
                }
                if (!overlaped)  // only add new key point if no overlap has been found in previous NMS
                {
                    keypoints.push_back(newKeyPoint);
                }
            }
        }
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "   Harris keypoints detection: " << keypoints.size() << " keypoints. Duration: " << 1000 * t / 1.0 << " ms"
         << endl;

    // visualize results
    if (bVis)
    {
        visualizeKeypoints(keypoints, img, "Harris");
    }
}

cv::Ptr<cv::FeatureDetector> initDetector(std::string detectorType)
{
    if (detectorType.compare("FAST") == 0)
    {
        int threshold = 30;  // difference between intensity of central pixel and pixels of circle around this pixel
        bool bNMS = true;    // perform non-maxima suppression on keypoints

        cv::FastFeatureDetector::DetectorType type =
            cv::FastFeatureDetector::TYPE_9_16;  // TYPE_9_16, TYPE_7_12, TYPE_5_8
        return cv::FastFeatureDetector::create(threshold, bNMS, type);
    }
    else if (detectorType.compare("BRISK") == 0)
    {
        return cv::BRISK::create();
    }
    else if (detectorType.compare("ORB") == 0)
    {
        return cv::ORB::create();
    }
    else if (detectorType.compare("AKAZE") == 0)
    {
        return cv::AKAZE::create();
    }
    else if (detectorType.compare("SIFT") == 0)
    {
        return cv::SIFT::create();
    }
    else
    {
        string error_message = "Unsupported detector type: " + detectorType;
        throw runtime_error(error_message);
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector = initDetector(detectorType);
    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "   " << detectorType.c_str() << " detection: " << keypoints.size()
         << " keypoints, duration: " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        visualizeKeypoints(keypoints, img, detectorType);
    }
}

string getDescriptorType(string descriptorName)
{
    return (descriptorName.compare("SIFT") == 0) ? "DES_HOG" : "DES_BINARY";
}