/* \author Andrew Krause */

#ifndef EUCLIDEAN_CLUSTER_H_
#define EUCLIDEAN_CLUSTER_H_

#include <unordered_set>
#include <pcl/common/common.h>
#include "kdtree.h"

template<typename PointT>
class EuclideanCluster {
    public:
        EuclideanCluster(int minSize, int maxSize, float distanceTol)
        : minSize(minSize), maxSize(maxSize), distanceTol(distanceTol)
        {}

        ~EuclideanCluster();

        std::vector<std::vector<int>> Cluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree);
    private:
        int minSize;
        int maxSize;
        float distanceTol;

        void clusterHelper(int indx, std::vector<int> &cluster, std::unordered_set<int> &processedPoints, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT> *tree);
};
#endif /* EUCLIDEAN_CLUSTER_H_ */
