
#ifndef camFusion_hpp
#define camFusion_hpp

#include <stdio.h>

#include <opencv2/core.hpp>
#include <vector>

#include "dataStructures.h"

void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints,
                         float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT);
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev,
                              std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches);
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame,
                        DataFrame &currFrame);

void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait = true);

void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg = nullptr);

/// @brief Computes time to collision to vehicle in front based on lidar measuremetns of current and previous frames
/// @details For computation of TTC the median of X coordinates for all lidar points of the frame within a ROI (bounding
/// box) is used. In this way the lidar measurement outliers are overcome during computation, that allows to get more
/// stable TTC value accors the frames sequence, although as a side effect this method introduces a slight permamnent
/// positive bias for calcualted TTC value for 'good' lidar points, since median of distances to these points is always
/// bit bigger than the distance to closest 'good' lidar point.
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev, std::vector<LidarPoint> &lidarPointsCurr,
                     double frameRate, double &TTC);
#endif /* camFusion_hpp */
