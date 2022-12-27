/* \author Andrew Krause */

#include "ransac.h"

template<typename PointT>
Ransac<PointT>::~Ransac() {}

template<typename PointT>
std::unordered_set<int> Ransac<PointT>::Run(typename pcl::PointCloud<PointT>::Ptr cloud)
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

		// coefficients for plane defined by above points (Ax + By + Cz + D = 0)
		double a = i;
		double b = j;
		double c = k;
		double d = -1 * (i * x1 + j * y1 + k * z1);


		// Measure distance between every point and fitted plane
        double norm = std::sqrt(a*a + b*b + c*c);
		for(int i = 0; i < cloud->points.size(); i++)
		{
			double x0 = cloud->points[i].x;
			double y0 = cloud->points[i].y;
			double z0 = cloud->points[i].z;

			double distance = std::abs(a*x0 + b*y0 + c*z0 + d) / norm;

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
