/* INCLUDES FOR THIS PROJECT */
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <sstream>
#include <vector>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/// @brief Process image set with using keypoint detector, descriptor and matcher
void processImages(const string& detectorType, const string& descriptorName, const string& matcherType)
{
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000";  // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0;  // first file index to load (assumes Lidar and camera names have same naming convention)
    int imgEndIndex = 9;    // last file index to load
    int imgFillWidth = 4;   // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;        // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer;  // list of data frames which are held in memory at the same time
    bool bVis = false;             // visualize results

    // only keep keypoints located in the image area of preceding vehicle
    bool bFocusOnVehicle = true;
    // optional : limit number of keypoints (helpful for debugging and learning)
    bool bLimitKpts = false;

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */
        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;

        if (dataBuffer.size() == dataBufferSize)
        {
            dataBuffer.erase(dataBuffer.begin());
        }
        dataBuffer.push_back(frame);

        std::cout << "Processing IMAGE " << imgNumber.str() << std::endl;

        /* DETECT IMAGE KEYPOINTS */
        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints;  // create empty feature list for current image

        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray);
        }
        else
        {
            detKeypointsModern(keypoints, imgGray, detectorType);
        }

        if (bFocusOnVehicle)
        {
            cv::Rect vehicleRect(535, 180, 180, 150);

            vector<cv::KeyPoint> filtered_keypoints;
            filtered_keypoints.reserve(keypoints.size());
            const auto isKeyPointInRect = [&vehicleRect](cv::KeyPoint keyPoint)
            { return keyPoint.pt.inside(vehicleRect); };
            std::copy_if(std::make_move_iterator(keypoints.begin()), std::make_move_iterator(keypoints.end()),
                         std::back_inserter(filtered_keypoints), isKeyPointInRect);
            keypoints = filtered_keypoints;
            if (bVis)
            {
                visualizeKeypoints(keypoints, imgGray, detectorType);
            }
            cout << "   Detected keypoints in ROI of preceding vehicle: " << keypoints.size() << endl;
        }

        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            {  // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        dataBuffer.rbegin()->keypoints = keypoints;
        // cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */
        cv::Mat descriptors;
        descKeypoints(dataBuffer.rbegin()->keypoints, dataBuffer.rbegin()->cameraImg, descriptors, descriptorName);

        // push descriptors for current frame to end of data buffer
        dataBuffer.rbegin()->descriptors = descriptors;

        // cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1)  // wait until at least two images have been processed
        {
            /* MATCH KEYPOINT DESCRIPTORS */
            vector<cv::DMatch> matches;

            string descriptorType = getDescriptorType(descriptorName);
            string selectorType = "SEL_KNN";  // SEL_NN, SEL_KNN

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors, matches,
                             descriptorType, matcherType, selectorType);

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "   Matching results: Detector: " << detectorType << ", Descriptor: " << descriptorName
                 << ", Matched descriptors: " << matches.size() << endl;

            // visualize matches between current and previous image
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints, matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(),
                                cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0);  // wait for key to be pressed
            }
        }
    }
}

int main(int argc, const char* argv[])
{
    // supported keypoint detector types
    vector<string> keypointDetectors{"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "SIFT", "AKAZE"};
    // supported keypoint decriptor types: "BRISK", "BRIEF", "ORB", "FREAK", "SIFT", "AKAZE"
    vector<string> descriptors{"BRISK", "BRIEF", "ORB", "FREAK", "SIFT"};
    // supported matcher types: "MAT_BF", "FLANN";
    string matcherType = "MAT_BF";

    for (const auto detector : keypointDetectors)
    {
        for (const auto descriptor : descriptors)
        {
            if (detector.compare("SIFT") == 0 && descriptor.compare("ORB") == 0)
            {
                continue;
            }
            processImages(detector, descriptor, matcherType);
        }
    }

    // AKAZE descriptor can work only with AKAZE keypoints
    processImages("AKAZE", "AKAZE", matcherType);
    return 0;
}