// PCL lib Functions for processing point clouds

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Voxel Grid Point Reduction
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> voxelGridFilter;
    voxelGridFilter.setInputCloud(cloud);
    voxelGridFilter.setLeafSize(filterRes, filterRes, filterRes);
    voxelGridFilter.filter(*cloudFiltered);

    // Region-based Filtering
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>());

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    // Remove roof points
    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (roofMinX, roofMinY, roofMinZ, 1));
    roof.setMax(Eigen::Vector4f (roofMaxX, roofMaxY, roofMaxZ, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices) {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    // extract obstacles from input cloud
    extract.setNegative(true);
    extract.filter(*obstCloud);

    // extract plane from input cloud
    extract.setNegative(false);
    extract.filter(*planeCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliers_set = Ransac(cloud, maxIterations, distanceThreshold);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    for (int index : inliers_set)
        inliers->indices.push_back(index);

    if(inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Create KdTree
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    // Extract clusters
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> extractor;
    extractor.setClusterTolerance(clusterTolerance);
    extractor.setMinClusterSize(minSize);
    extractor.setMaxClusterSize(maxSize);
    extractor.setSearchMethod(tree);
    extractor.setInputCloud(cloud);
    extractor.extract(clusterIndices);

    // Convert cluster indicies to point clouds
    for (const auto& cluster : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for (const auto& idx : cluster.indices)
        {
            cloudCluster->points.push_back(cloud->points[idx]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
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
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}


template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations
	while(maxIterations--)
	{
		std::unordered_set<int> inliers;

		// Randomly sample subset and fit plane
		while(inliers.size() < 3)
		{
			inliers.insert(rand() % cloud->size());
		}

		auto iter = inliers.begin();
		double x1 = cloud->points[*iter].x;
		double y1 = cloud->points[*iter].y;
		double z1 = cloud->points[*iter].z;
		iter++;
		double x2 = cloud->points[*iter].x;
		double y2 = cloud->points[*iter].y;
		double z2 = cloud->points[*iter].z;
		iter++;
		double x3 = cloud->points[*iter].x;
		double y3 = cloud->points[*iter].y;
		double z3 = cloud->points[*iter].z;

		// coordinates of normal vector to the plane defined by above points
		double i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		double j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		double k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

		// coefficients for plane defined via Ax + By + Cz + D = 0
		double a = i;
		double b = j;
		double c = k;
		double d = -1 * (i * x1 + j * y1 + k * z1);


		// Measure distance between every point and fitted plane
		for(int i = 0; i < cloud->points.size(); i++)
		{
			double x0 = cloud->points[i].x;
			double y0 = cloud->points[i].y;
			double z0 = cloud->points[i].z;

			double distance = std::abs(a * x0 + b * y0 + c * z0 + d) / std::sqrt(a*a + b*b + c*c);

			// If distance is smaller than threshold count it as inlier
			if(distance <= distanceTol)
			{
				inliers.insert(i);
			}
		}

		// If plane is a better fit (has more inliers) than our current best result, then save it
		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

	// Return indicies of inliers from fitted plane with most inliers
	return inliersResult;
}
