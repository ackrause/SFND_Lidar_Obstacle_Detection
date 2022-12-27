/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::tuple<double, double, double> linearCoefficients(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
	double a = p1.y - p2.y;
	double b = p2.x - p1.x;
	double c = (p1.x * p2.y) - (p2.x * p1.y);

	return std::make_tuple(a, b, c);
}

double distanceToLine(pcl::PointXYZ p, double a, double b, double c)
{
	return std::abs(a * p.x + b * p.y + c) / std::sqrt(a * a + b * b);
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations 
	for(int iter = 0; iter < maxIterations; iter++)
	{
		std::unordered_set<int> inliers;

		// Randomly sample subset and fit line
		int index1 = rand() % cloud->size();
		int index2 = rand() % cloud->size();

		while(index2 == index1)
		{
			index2 = rand() % cloud->size();
		}

		double a, b, c;
		std::tie(a, b, c) = linearCoefficients(cloud->points[index1], cloud->points[index2]);

		// Measure distance between every point and fitted line
		for(int i = 0; i < cloud->points.size(); i++)
		{
			// If distance is smaller than threshold count it as inlier
			if(distanceToLine(cloud->points[i], a, b, c) <= distanceTol)
			{
				inliers.insert(i);
			}
		}

		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliers = Ransac3D(cloud, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
