#include "point_cloud_processor.h"
#include "render.h"
// using templates for point_cloud_processor so also include .cpp to help linker
#include "point_cloud_processor.cpp"

/// @brief Opens 3D viewer and renders City Block from input point cloud
void renderCityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
                     PointCloudProcessor<pcl::PointXYZI>* point_processor,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
{
    // input point cloud filtering constants
    const float kFilterResolution = 0.17F;  // in meters
    const float kFilterBBoxMinX = -8.F;     // in meters
    const float kFilterBBoxMinY = -6.F;     // in meters
    const float kFilterBBoxMinZ = -2.F;     // in meters
    const float kFilterBBoxMaxX = 20.F;     // in meters
    const float kFilterBBoxMaxY = 6.F;      // in meters
    const float kFilterBBoxMaxZ = 3.F;      // in meters

    // plane segmentation constants
    const uint8_t kSegmentationMaxIterations = 25U;
    const float kSegmentationDistanceThreshold = 0.16F;  // in meters

    // obstacle point clouds clastering constants
    const float kClusteringDistanceThreshold = 0.44575F;
    const uint16_t kClusteringMinSize = 17U;
    const uint16_t kClusteringMaxSize = 1000U;

    // FILTER POINT CLOUD
    // const auto& filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.15f , Eigen::Vector4f (-10.f, -6.3f,
    // -2.2f, 1), Eigen::Vector4f ( 30.f, 6.3f, 2.f, 1));
    const auto& filtered_cloud = point_processor->filterCloud(
        input_cloud, kFilterResolution, Eigen::Vector4f(kFilterBBoxMinX, kFilterBBoxMinY, kFilterBBoxMinZ, 1),
        Eigen::Vector4f(kFilterBBoxMaxX, kFilterBBoxMaxY, kFilterBBoxMaxZ, 1));

    // CLOUD SEGMENTATION
    const auto& segmented_clouds =
        point_processor->segmentPlane(filtered_cloud, kSegmentationMaxIterations, kSegmentationDistanceThreshold);

    // PCL CLUSTERING
    const auto& cloud_clusters = point_processor->clustering(segmented_clouds.first, kClusteringDistanceThreshold,
                                                             kClusteringMinSize, kClusteringMaxSize);

    // RENDER POINT CLOUD(S)
    // redefine color palete for obstacles
    const std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1), Color(0, 1, 1), Color(1, 0, 1)};

    int cluster_id = 0;
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloud_clusters)
    {
        std::cout << "cluster size ";
        point_processor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(cluster_id), colors[cluster_id % colors.size()]);

        const auto box = point_processor->createBoundingBox(cluster);
        renderBox(viewer, box, cluster_id);

        ++cluster_id;
    }

    // render sergmented road surface in green color
    renderPointCloud(viewer, segmented_clouds.second, "planeCloud", Color(0, 1, 0));
}

/// @author Aaron Brown
/// @param setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
///
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
        case XY:
            viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
            break;
        case TopDown:
            viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
            break;
        case Side:
            viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
            break;
        case FPS:
            viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS) viewer->addCoordinateSystem(1.0);
}

int main(int argc, char** argv)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    const auto point_processor = new PointCloudProcessor<pcl::PointXYZI>();

    // PROCESSSOR FOR SINGLE PCD FILE
    // const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud =
    // pointProcessorI->LoadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd"); RenderCityBlock(viewer,
    // pointProcessorI, inputCloud); while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce ();
    // }

    // PROCESSSOR FOR SEQUENCE OF PCD FILES
    // read folder with pcd files as vector of file paths
    std::vector<boost::filesystem::path> stream = point_processor->streamPcd("../resources/pcd/data_1");
    auto stream_iterator = stream.begin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;
    while (!viewer->wasStopped())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        input_cloud = point_processor->loadPcd((*stream_iterator).string());
        renderCityBlock(viewer, point_processor, input_cloud);

        stream_iterator++;
        if (stream_iterator == stream.end())
        {
            stream_iterator = stream.begin();
        }

        viewer->spinOnce();
    }
}