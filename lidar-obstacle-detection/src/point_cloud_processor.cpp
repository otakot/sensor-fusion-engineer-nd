// PCL lib Functions for processing point clouds 

#include "point_cloud_processor.h"

#include "kdtree.h"

#include <chrono>
#include <ctime>
#include <iostream>
#include <pcl/common/pca.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <utility>

//constructor:
template<typename PointT>
PointCloudProcessor<PointT>::PointCloudProcessor() {}

//de-constructor:
template<typename PointT>
PointCloudProcessor<PointT>::~PointCloudProcessor() {}

template<typename PointT>
void PointCloudProcessor<PointT>::numPoints(PointCloudPtr<PointT> cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
PointCloudPtr<PointT> PointCloudProcessor<PointT>::filterCloud(
                PointCloudPtr<PointT> cloud, float filter_resolution,
                            Eigen::Vector4f min_point, Eigen::Vector4f max_point)
{
    // Time segmentation process
    auto start_time = std::chrono::steady_clock::now();

    // VOXEL GRID POINT REDUCTION
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filter_resolution, filter_resolution, filter_resolution);
    PointCloudPtr<PointT> cloud_filtered(new typename pcl::PointCloud<PointT>());
    sor.filter (*cloud_filtered);

    // REGION BASED FILTERGRID
    //crop all cloud points outside of roi
    pcl::CropBox<PointT> roi_crop_box(true);
    roi_crop_box.setMin(min_point);
    roi_crop_box.setMax(max_point);
    roi_crop_box.setInputCloud(cloud_filtered);
    PointCloudPtr<PointT> filtered_cloud_roi(new typename pcl::PointCloud<PointT>());
    roi_crop_box.filter(*filtered_cloud_roi);

    // crop points in close proximilty to lidar (ego car roof surface occlusions)
    pcl::CropBox<PointT> roof_crop_box(true);
    roof_crop_box.setMin(Eigen::Vector4f(-1.5, -1.7, -1.1, 1));
    roof_crop_box.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof_crop_box.setInputCloud(filtered_cloud_roi);
    std::vector<int> roof_point_ids;
    roof_crop_box.filter(roof_point_ids);

    pcl::PointIndices::Ptr roof_points(new pcl::PointIndices);
    std::move(std::begin(roof_point_ids), std::end(roof_point_ids), std::back_inserter(roof_points->indices));

    pcl::ExtractIndices<PointT> roof_points_extractor;
    roof_points_extractor.setInputCloud(filtered_cloud_roi);
    roof_points_extractor.setIndices(roof_points);
    roof_points_extractor.setNegative(true);
    roof_points_extractor.filter(*filtered_cloud_roi);

    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "filtering took " << elapsed_time.count() << " milliseconds" << std::endl;

    return filtered_cloud_roi;
}

template<typename PointT>
std::pair<PointCloudPtr<PointT>, PointCloudPtr<PointT>> PointCloudProcessor<PointT>::separateClouds(
    std::unordered_set<std::size_t> inlier_indices, PointCloudPtr<PointT> cloud)
{
    // Extract the plane points from original pointcloud
    PointCloudPtr<PointT> plane_point_cloud (new  pcl::PointCloud<PointT>());
    PointCloudPtr<PointT> obstacle_point_cloud (new pcl::PointCloud<PointT>());
    for (size_t point_idx = 0; point_idx < cloud->points.size(); point_idx++)
    {
        if(inlier_indices.count(point_idx)>0) {
            plane_point_cloud->points.push_back(cloud->points[point_idx]);
        } else {
            obstacle_point_cloud->points.push_back(cloud->points[point_idx]);
        }
    }
    return std::make_pair(obstacle_point_cloud, plane_point_cloud);
}


template<typename PointT>
std::pair<PointCloudPtr<PointT>, PointCloudPtr<PointT>> PointCloudProcessor<PointT>::segmentPlane(
    PointCloudPtr<PointT> cloud, uint8_t max_iterations, float distance_threshold)
{
    // Time segmentation process
    auto start_time = std::chrono::steady_clock::now();

    std::unordered_set<std::size_t> inliers_result;
	while (max_iterations--)
	{
		// pick random 3 points
		std::unordered_set<std::size_t> inliers;
		while (inliers.size() < 3)
		{
			inliers.insert(rand()%(cloud->points.size()));
		}

		// build the plane through 3 random points from data set
		// gereral equation of plane through 3 points: Ax+By+Cz+D=0, where
		// A = (y2−y1)(z3−z1)−(z2−z1)(y3−y1)
		// B = (z2−z1)(x3−x1)−(x2−x1)(z3−z1)
		// C = (x2−x1)(y3−y1)−(y2−y1)(x3−x1)
		// D = -(Ax1 + By1 + Cz1)

		auto itr = inliers.begin();
        const auto point_1_idx = *itr++;
        const auto& point_1 = cloud->points[point_1_idx];
        const auto point_2_idx = *itr++;
        const auto& point_2 = cloud->points[point_2_idx];
        const auto point_3_idx = *itr;
        const auto& point_3 = cloud->points[point_3_idx];

        float y2_sub_y1 = point_2.y - point_1.y;
        float z3_sub_z1 = point_3.z - point_1.z;
        float z2_sub_z1 = point_2.z - point_1.z;
        float y3_sub_y1 = point_3.y - point_1.y;
        float x3_sub_x1 = point_3.x - point_1.x;
        float x2_sub_x1 = point_2.x - point_1.x;

        float a = y2_sub_y1 * z3_sub_z1 - z2_sub_z1 * y3_sub_y1;
		float b = z2_sub_z1 * x3_sub_x1 - x2_sub_x1 * z3_sub_z1;
		float c = x2_sub_x1 * y3_sub_y1 - y2_sub_y1 * x3_sub_x1;
		float d = -a * point_1.x - b * point_1.y - c * point_1.z;

		// iterate over the points of data set and find those of them closer to the plane than provided threshold
		// distance from point to plane: |A*x+B*y+C*z+D|/sqrt(A^2+B^2+C^2)
		for (std::size_t index = 0; index < cloud->points.size(); index ++)
		{
			// skip points used for building the plane
			if(index == point_3_idx || index == point_2_idx || index == point_1_idx)
            {
				continue;
			}
			const auto& point = cloud->points[index];
			float dist = fabs(a * point.x + b * point.y + c * point.z + d) / sqrt(a * a + b * b + c * c);
			if(dist < distance_threshold)
			{
				inliers.insert(index);
			}
		}

		// check for the best inliers update need
		if(inliers.size() > inliers_result.size()) {
			inliers_result = inliers;
		}
	}

    const auto& seg_result = separateClouds(inliers_result, cloud);

    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Plane segmentation took " << elapsed_time.count() << " milliseconds" << std::endl;

    return seg_result;
}

namespace
{
template<typename PointT>
std::vector<int> findClusterPoints(int cluster_seed_point_idx, PointCloudPtr<PointT> const& cloud,
     KdTree<PointT>* const& tree, std::vector<bool>& processed_point_ids, float distanceTol)
{
	std::vector<int> cluster_of_point_ids{cluster_seed_point_idx};
	const auto nearby_point_ids = tree->search(cloud->points[cluster_seed_point_idx], distanceTol);
    processed_point_ids[cluster_seed_point_idx] = true;
	for (const auto& nearby_point_idx : nearby_point_ids)
	{
		if(!processed_point_ids[nearby_point_idx]) {
			const auto transitive_cluster_point_ids = findClusterPoints(nearby_point_idx, cloud, tree, processed_point_ids, distanceTol);
			std::move(transitive_cluster_point_ids.begin(), transitive_cluster_point_ids.end(),  std::back_inserter(cluster_of_point_ids));
		}
	}
	return cluster_of_point_ids;
}

template<typename PointT>
std::vector<std::vector<int>> euclideanClustering(PointCloudPtr<PointT> const& cloud,
     KdTree<PointT>* const& tree, float distanceTol, uint16_t min_size, uint16_t max_size)
{
	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed_point_ids(cloud->points.size(), false);
	for (int point_idx = 0; point_idx < cloud->points.size(); point_idx++)
	{
		if(!processed_point_ids[point_idx])
		{
			std::vector<int> cluster_of_points = findClusterPoints(point_idx, cloud, tree, processed_point_ids, distanceTol);
            if(cluster_of_points.size() >= min_size &&  cluster_of_points.size() <= max_size)
            {
			    clusters.emplace_back(std::move(cluster_of_points));
            }
		}
	}
	return clusters;
}
} //namespace

template<typename PointT>
std::vector<PointCloudPtr<PointT>> PointCloudProcessor<PointT>::clustering(
    PointCloudPtr<PointT> cloud, float cluster_tolerance, uint16_t min_size, uint16_t max_size)
{
    // Start timer of clustering process
    auto start_ime = std::chrono::steady_clock::now();

    // populate KD tree with cloud points
    KdTree<PointT>* tree = new KdTree<PointT>();
    for (size_t i = 0; i < cloud->points.size(); i++) 
    {
    	tree->insert(cloud->points[i],i);
    }

    // detect clusters of points
    const auto& clustered_point_ids = euclideanClustering(cloud, tree, cluster_tolerance, min_size, max_size);

    // create result
    std::vector<PointCloudPtr<PointT>> clusters;
    for(std::vector<int> cluster_points : clustered_point_ids)
  	{
  		PointCloudPtr<PointT> cluster(new pcl::PointCloud<PointT>());
  		for(int point_idx : cluster_points)
        {
  			cluster->points.push_back(cloud->points[point_idx]);
        }
        clusters.emplace_back(std::move(cluster));
    }

    // Stop timer of clustering process
    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_ime);
    std::cout << "Clustering took " << elapsed_time.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box PointCloudProcessor<PointT>::createBoundingBox(PointCloudPtr<PointT> cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
BoxQ PointCloudProcessor<PointT>::createBoundingBoxQ(PointCloudPtr<PointT> cluster)
{
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PCA<pcl::PointXYZ> pca;
    // pca.setInputCloud(cluster);
    // pca.project(*cluster, *cloudPCAprojection);

    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); 

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // create bounding box struct
    BoxQ box;
    box.bboxQuaternion = Eigen::Quaternionf{eigenVectorsPCA /* pca.getEigenVectors() */};
    box.bboxTransform =  eigenVectorsPCA /* pca.getEigenVectors() */ * meanDiagonal + pcaCentroid.head<3>();
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;
    return box;
}


template<typename PointT>
PointCloudPtr<PointT> PointCloudProcessor<PointT>::loadPcd(std::string file)
{
    PointCloudPtr<PointT> cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> PointCloudProcessor<PointT>::streamPcd(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());
    return paths;
}