#include "ransac2d.h"

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
  std::unique_ptr<std::unordered_set<int>> p_inliers =std::make_unique<std::unordered_set<int>>();
	// For max iterations
  for (int i = 0; i < maxIterations; i++){
	  // Randomly sample subset and fit line
    int i1 = (double)rand()/RAND_MAX*cloud->points.size();
    int i2 =  (double)rand()/RAND_MAX*cloud->points.size();
    pcl::PointXYZ p1 = cloud->points[i1];
    pcl::PointXYZ p2 = cloud->points[i2];
    double a = p1.y - p2.y;
    double b = p2.x - p1.x;
    double c = p1.x*p2.y - p1.y*p2.x;
    std::unique_ptr<std::unordered_set<int>> cur_inliers = std::make_unique<std::unordered_set<int>>();
	  // Measure distance between every point and fitted line
    for (int ip = 0; ip < cloud->points.size(); ip++){
      if ((ip == i1) || (ip == i2))
        continue;
      auto p = cloud->points[ip];
      double dist = abs(a*p.x + b*p.y + c)/sqrt(a*a + b*b);
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
