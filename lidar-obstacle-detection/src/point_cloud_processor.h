/// @brief PCL lib Functions for processing point clouds

#pragma once

#include <pcl/common/common.h>

#include <boost/filesystem/operations.hpp>
#include <unordered_set>
#include <vector>

#include "box.h"

template <typename PointType>
using PointCloudPtr = typename pcl::PointCloud<PointType>::Ptr;

template <typename PointT>
class PointCloudProcessor
{
   public:
    // constructor
    PointCloudProcessor();
    // deconstructor
    ~PointCloudProcessor();

    void numPoints(PointCloudPtr<PointT> cloud);

    PointCloudPtr<PointT> filterCloud(PointCloudPtr<PointT> cloud, float filterRes, Eigen::Vector4f minPoint,
                                      Eigen::Vector4f maxPoint);

    std::pair<PointCloudPtr<PointT>, PointCloudPtr<PointT>> separateClouds(
        std::unordered_set<std::size_t> inlier_indices, PointCloudPtr<PointT> cloud);

    std::pair<PointCloudPtr<PointT>, PointCloudPtr<PointT>> segmentPlane(PointCloudPtr<PointT> cloud,
                                                                         uint8_t maxIterations,
                                                                         float distanceThreshold);

    std::vector<PointCloudPtr<PointT>> clustering(PointCloudPtr<PointT> cloud, float clusterTolerance, uint16_t minSize,
                                                  uint16_t maxSize);

    /// @author Aaron Brown
    Box createBoundingBox(PointCloudPtr<PointT> cluster);

    /// @author Nicola Fioraio
    BoxQ createBoundingBoxQ(PointCloudPtr<PointT> cluster);

    /// @author Aaron Brown
    void savePcd(PointCloudPtr<PointT> cloud, std::string file);

    /// @author Aaron Brown
    PointCloudPtr<PointT> loadPcd(std::string file);

    /// @author Aaron Brown
    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};