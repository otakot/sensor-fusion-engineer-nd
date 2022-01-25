#pragma once

#include <stdio.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "dataStructures.h"

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis = false);

void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis = false);

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType,
                        bool bVis = false);

void descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors,
                   std::string descriptorType);

void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource,
                      cv::Mat &descRef, std::vector<cv::DMatch> &matches, std::string descriptorType,
                      std::string matcherType, std::string selectorType);

void visualizeKeypoints(const std::vector<cv::KeyPoint> &keypoints, const cv::Mat &img, std::string detectorType);

cv::Ptr<cv::FeatureDetector> initDetector(std::string detectorType);

/// @brief Returns the type of keypoint descriptor (bi) for given decriptior name
/// @details Supported decriptor types: binary and HOG based
/// @return type of descriptor. Supported values: DES_BINARY, DES_HOG
std::string getDescriptorType(std::string descriptorName);
