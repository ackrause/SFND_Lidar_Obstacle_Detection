/* \author Andrew Krause */

#ifndef RANSAC_H_
#define RANSAC_H_

#include <pcl/common/common.h>
#include <unordered_set>

template<typename PointT>
class Ransac {
    public:
        Ransac(int maxIter, float distTol)
        : maxIterations(maxIter), distanceTol(distTol)
        {}

        ~Ransac();

        std::unordered_set<int> Run(typename pcl::PointCloud<PointT>::Ptr cloud);
    private:
        int maxIterations;
        float distanceTol;
};
#endif /* RANSAC_H */
