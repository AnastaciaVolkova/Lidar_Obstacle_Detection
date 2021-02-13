#include <unordered_set>
#include "../../processPointClouds.h"

#ifndef RANSAC2D_H_
#define RANSAC2D_H_
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol);

template <typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
  std::unique_ptr<std::unordered_set<int>> p_inliers =std::make_unique<std::unordered_set<int>>();
	// For max iterations
  for (int i = 0; i < maxIterations; i++){
	  // Randomly sample subset and fit line
    int i1 = (double)rand()/RAND_MAX*cloud->points.size();
    int i2 =  (double)rand()/RAND_MAX*cloud->points.size();
    int i3 =  (double)rand()/RAND_MAX*cloud->points.size();

    PointT p1 = cloud->points[i1];
    PointT p2 = cloud->points[i2];
    PointT p3 = cloud->points[i3];

    double a = (p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y); 
    double b = (p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z); 
    double c = (p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x); 
    double d = -(a*p1.x+b*p1.y+c*p1.z);
 
    std::unique_ptr<std::unordered_set<int>> cur_inliers = std::make_unique<std::unordered_set<int>>();
	  // Measure distance between every point and fitted line
    for (int ip = 0; ip < cloud->points.size(); ip++){
      if ((ip == i1) || (ip == i2))
        continue;
      auto p = cloud->points[ip];
      double dist = abs(a*p.x + b*p.y + c*p.z+d)/sqrt(a*a + b*b + c*c);
      if (dist < distanceTol)
        cur_inliers->insert(ip);
    }
    if (cur_inliers->size() > p_inliers->size()){
      p_inliers = std::move(cur_inliers);
    }
  }

	inliersResult = *p_inliers;
  return inliersResult;
}

#endif

