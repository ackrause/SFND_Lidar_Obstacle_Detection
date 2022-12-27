#include "euclidean_cluster.h"

template<typename PointT>
EuclideanCluster<PointT>::~EuclideanCluster() {}


template<typename PointT>
std::vector<std::vector<int>> EuclideanCluster<PointT>::Cluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree)
{
	std::vector<std::vector<int>> clusters;
	std::unordered_set<int> processedPoints;

	for (int indx = 0; indx < cloud->size(); indx++)
	{
		if (processedPoints.count(indx) > 0)
			continue;

		std::vector<int> cluster;
		clusterHelper(indx, cluster, processedPoints, cloud, tree);

        if (cluster.size() >= minSize && cluster.size() <= maxSize)
		    clusters.push_back(cluster);
	}

	return clusters;
}


template<typename PointT>
void EuclideanCluster<PointT>::clusterHelper(int indx, std::vector<int> &cluster, std::unordered_set<int> &processedPoints, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT> *tree)
{
	processedPoints.insert(indx);

	cluster.push_back(indx);

	std::vector<int> nearbyPoints = tree->search(cloud->points[indx], distanceTol);

	for (int point : nearbyPoints)
	{
		if (processedPoints.count(point) > 0)
		{
			continue;
		}

		clusterHelper(point, cluster, processedPoints, cloud, tree);
	}
}
