
#include <algorithm>
#include <cassert>
//#include <cmath>
#include <iostream>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unordered_map>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

namespace
{
double getMedian(std::vector<double> data)
{
    const auto start = data.begin();
    const auto end = data.end();

    // Find  two adjacent elements in the middle of vector (they will be the same if vector size is odd)
    const auto left_median_ptr = start + (data.size() - 1) / 2;
    const auto right_median_ptr = start + data.size() / 2;

    // Partial sort to place correct elements of vector at median indexes
    std::nth_element(start, left_median_ptr, end);
    std::nth_element(left_median_ptr, right_median_ptr, end);

    return 0.5 * (*left_median_ptr + *right_median_ptr);
}

double getStdDeviation(const std::vector<double> &data, double mean)
{
    double variance{0};
    for (const auto &elem : data)
    {
        variance += (elem - mean) * (elem - mean);
    }
    variance = variance / (double)data.size();
    return sqrt(variance);
}

double computeEuclideanDistance(cv::Point2f rhs, cv::Point2f lhs)
{
    double dx = rhs.x - lhs.x;
    double dy = rhs.y - lhs.y;
    return std::sqrt((dx * dx) + (dy * dy));
}

}  // namespace

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints,
                         float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);

        vector<vector<BoundingBox>::iterator>
            enclosingBoxes;  // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.emplace_back(it2);
            }

        }  // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        {
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.emplace_back(*it1);
        }

    }  // eof loop over all Lidar points
}

void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for (auto it1 = boundingBoxes.begin(); it1 != boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
        float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x;  // world position in m with x facing forward from sensor
            float yw = (*it2).y;  // world position in m with y facing left from sensor
            xwmin = xwmin < xw ? xwmin : xw;
            ywmin = ywmin < yw ? ywmin : yw;
            ywmax = ywmax > yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top < y ? top : y;
            left = left < x ? left : x;
            bottom = bottom > y ? bottom : y;
            right = right > x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left + 5, bottom - 55), cv::FONT_ITALIC, 1, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
        putText(topviewImg, str2, cv::Point2f(left + 5, bottom - 22), cv::FONT_ITALIC, 1, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0;  // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
        putText(topviewImg, std::to_string((int)(i * lineSpacing)) + "m", cv::Point(5, y - 10), cv::FONT_ITALIC, 1,
                cv::Scalar(255, 0, 0));
        putText(topviewImg, std::to_string((int)(i * lineSpacing)) + "m", cv::Point(imageSize.width - 80, y - 10),
                cv::FONT_ITALIC, 1, cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if (bWait)
    {
        cv::waitKey(0);  // wait for key to be pressed
    }
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev,
                              std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    std::vector<double> matchedKptDistances;
    std::vector<std::vector<cv::DMatch>::iterator> eclosedMatches;
    for (auto kptMatch = kptMatches.begin(); kptMatch != kptMatches.end(); ++kptMatch)
    {
        const auto &currKpt = kptsCurr[kptMatch->trainIdx];
        if (boundingBox.roi.contains(currKpt.pt))
        {
            eclosedMatches.emplace_back(kptMatch);
            const auto &prevKpt = kptsPrev[kptMatch->queryIdx];

            // calculate euclidean distance [in pixels] between matched keypoints
            double dist = computeEuclideanDistance(currKpt.pt, prevKpt.pt);
            matchedKptDistances.emplace_back(dist);
        }
    }

    // double distanceMean = std::accumulate(matchedKptDistances.begin(), matchedKptDistances.end(), 0.0) /
    //                       (double)matchedKptDistances.size();
    double distanceMedian = getMedian(matchedKptDistances);  // median allowes to disqualify outliers better than mean
    double sigma = getStdDeviation(matchedKptDistances, distanceMedian);

    for (int i = 0; i < eclosedMatches.size(); i++)
    {
        // toleration of sigma/4 deviation from median distance value across all matched keypoints
        if (abs(matchedKptDistances[i] - distanceMedian) <= sigma / 10)
        {
            boundingBox.keypoints.emplace_back(kptsCurr[eclosedMatches[i]->trainIdx]);
            boundingBox.kptMatches.emplace_back(*(eclosedMatches[i]));
        }
    }
    cout << "   Cluster Keypoints results for bBox ID " << boundingBox.boxID
         << ": eclosed matches: " << eclosedMatches.size() << ", Euclidean distance median: " << distanceMedian
         << "px, StDev: " << sigma << "px, qualified matches: " << boundingBox.kptMatches.size() << endl;
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    vector<double> distRatios;  // distance ratios for all meaningful keypoint pairs between current and previous frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    {
        // get current keypoint and its matched partner in the previous frame
        const auto kptOuterCurr = kptsCurr.at(it1->trainIdx);
        const auto kptOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        {
            // min. tolerated distance [in px] between keypoints in the frame
            // which will allow to properly calculate TTC
            double minDistance = 110.0;
            // get next keypoint and its matched partner in the previous frame
            const auto kptInnerCurr = kptsCurr.at(it2->trainIdx);
            const auto kptInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances between same key point pairs in both current and previous frames
            // and ratio between these distances to identify the camera image scale change
            double distCurr = computeEuclideanDistance(kptOuterCurr.pt, kptInnerCurr.pt);
            double distPrev = computeEuclideanDistance(kptOuterPrev.pt, kptInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDistance)
            {
                double distRatio = distCurr / distPrev;
                distRatios.emplace_back(distRatio);
            }
        }
    }
    // exit if list of distance ratios is empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }
    // compute camera-based TTC based on distance ratios
    double medianDistRatio = getMedian(distRatios);
    double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();
    double dT = 1 / frameRate;  // time [in seconds] between camera measurements (images)

    // if true, then median ratio values will be used for TTC calculation instead of mean
    bool useMedianDistRatioForTTC = true;
    double medianDistRatioTTC = -dT / (1 - medianDistRatio);
    double meanDistRatioTTC = -dT / (1 - meanDistRatio);
    cout << "   Camera TCC results: Median distance ratio based TTC: " << medianDistRatioTTC
         << "s, Mean distance ratio based TTC: " << meanDistRatioTTC << "s" << endl;
    TTC = useMedianDistRatioForTTC ? medianDistRatioTTC : meanDistRatioTTC;
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev, std::vector<LidarPoint> &lidarPointsCurr,
                     double frameRate, double &TTC)
{
    double dT = 1 / frameRate;  // time between two measurements in seconds

    // get distances to Lidar points
    vector<double> Xprev;
    std::transform(lidarPointsPrev.begin(), lidarPointsPrev.end(), std::back_inserter(Xprev),
                   [](LidarPoint point) -> double { return point.x; });
    std::sort(Xprev.begin(), Xprev.end());
    double minXPrev = Xprev.front();
    double maxXPrev = Xprev.back();

    vector<double> Xcurr;
    std::transform(lidarPointsCurr.begin(), lidarPointsCurr.end(), std::back_inserter(Xcurr),
                   [](LidarPoint point) -> double { return point.x; });
    std::sort(Xcurr.begin(), Xcurr.end());
    double minXCurr = Xcurr.front();
    double maxXCurr = Xcurr.back();

    // median X value of lidar measuremetns of current frame (! not mean, to be more robust to outliers)
    double medianXCurr = getMedian(Xcurr);
    // median X value of lidar measuremetns of previous frame (! not mean, to be more robust to outliers)
    double medianXPrev = getMedian(Xprev);
    // standard deviation for X value of lidar measuremetns of current frame
    // double sigmaXCurr = getStdDeviation(Xcurr, medianXCurr);
    // standard deviation for X value of lidar measuremetns of previous frame
    // double sigmaXPrev = getStdDeviation(Xprev, medianXPrev);
    cout << "   Curr frame: Min X: " << minXCurr << "m, Max X: " << maxXCurr << "m, Median X: " << medianXCurr << endl;
    //     << "m, Sigma: " << sigmaXCurr << "m" << endl;
    cout << "   Prev frame: Min X: " << minXPrev << "m, Max X: " << maxXPrev << "m, Median X: " << medianXPrev << endl;
    //    << "m, Sigma: " << sigmaXPrev << "m" << endl;

    // compute TTC based on calculated distances to vehicle in front for previous and current frames
    bool useMedianXforTTC = true;  // if true, then median X values will be used for TTC calculation instead of min X
    double minXTTC = minXCurr * dT / (minXPrev - minXCurr);
    double medianXTTC = medianXCurr * dT / (medianXPrev - medianXCurr);
    TTC = (useMedianXforTTC) ? medianXTTC : minXTTC;
    cout << "   Lidar TCC results: Median X based TTC: " << medianXTTC << "s, Min X based TTC: " << minXTTC << "s,"
         << endl;
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame,
                        DataFrame &currFrame)

{
    // idea is to iterate over all bounding boxes of previous frame
    // and find best correspondence for each of them in ciurrent frame
    for (const auto &bBoxPrev : prevFrame.boundingBoxes)
    {
        // list of matches, which target keypoints (in previous frame) are eclosed by this bounding box
        std::vector<std::vector<cv::DMatch>::iterator> bBoxPrevEclosedMatches;
        for (auto kptMatchPtr = matches.begin(); kptMatchPtr != matches.end(); ++kptMatchPtr)
        {
            assert(prevFrame.keypoints.size() > kptMatchPtr->queryIdx);
            const auto &keyPoint = prevFrame.keypoints.at(kptMatchPtr->queryIdx);
            // check wether keypoint is within bounding box of previous frame
            if (bBoxPrev.roi.contains(keyPoint.pt))
            {
                bBoxPrevEclosedMatches.push_back(kptMatchPtr);
            }
        }

        // IDs of bounding Boxes in current frame that enclose
        // source keypoint matches identified above
        std::unordered_map<int, int> bBoxCurrMatchCandidates;
        for (const auto &bBoxCurr : currFrame.boundingBoxes)
        {
            for (const auto enclosedMatch : bBoxPrevEclosedMatches)
            {
                assert(currFrame.keypoints.size() > enclosedMatch->trainIdx);
                const auto &keyPoint = currFrame.keypoints.at(enclosedMatch->trainIdx);
                // check whether keypoint is within this bounding box
                if (bBoxCurr.roi.contains(keyPoint.pt))
                {
                    bBoxCurrMatchCandidates[bBoxCurr.boxID] += 1;
                }
            }
        }
        if (bBoxCurrMatchCandidates.empty())
        {
            string error_message =
                "No matching bounding box found in current frame "
                "for corresponding bounding box of previous frame with ID: " +
                std::to_string(bBoxPrev.boxID);
            throw runtime_error(error_message);
        }

        // uncomment this block for debugging purposes
        // for (auto x : bBoxPrevMatchCandidates)
        // {
        //     cout << "   match candicate: boxID: " << x.first << ", "
        //          << ", keypoints: " << x.second << endl;
        // }

        auto bestMatchBBoxCurr = std::max_element(bBoxCurrMatchCandidates.begin(), bBoxCurrMatchCandidates.end(),
                                                  [](const std::pair<int, int> &a, const std::pair<int, int> &b) -> bool
                                                  { return a.second < b.second; });
        std::cout << "  bBox match results: previous bBox ID: " << bBoxPrev.boxID
                  << ", enclosed keypoints: " << bBoxPrevEclosedMatches.size()
                  << ", current bBoxId: " << bestMatchBBoxCurr->first
                  << ", enclosed keypoints: " << bestMatchBBoxCurr->second << std::endl;

        bbBestMatches[bBoxPrev.boxID] = bestMatchBBoxCurr->first;
    }
}
