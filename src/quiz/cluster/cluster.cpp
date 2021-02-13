#include "cluster.h"

// Function returns list of indices for each cluster
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
	// Clusters of points.
	std::vector<std::vector<int>> clusters;

	// Indicator if point was visited.
	std::vector<bool> is_visited(points.size(), false);

	int cur_point_ix; // Current point index.
	std::vector<int> cluster_i; // Cluster for current point.
	std::set<int> neighbours; // Neighbours of current point
	std::vector<int> nt; // Temporary storage for neighbours of neighbour.
	
	for (int i = 0; i < points.size(); i++){
		if (is_visited[i]) continue;
		cur_point_ix = i;
		is_visited[cur_point_ix] = true;
		cluster_i.push_back(cur_point_ix);
		nt = tree->search(points[cur_point_ix], distanceTol);
		std::copy(nt.begin(), nt.end(), std::inserter(neighbours, neighbours.begin()));
	
		while (neighbours.size() != 0){
			cur_point_ix = *neighbours.begin();
			neighbours.erase(*neighbours.begin());
			if (is_visited[cur_point_ix])
				continue;
			is_visited[cur_point_ix] = true;
			cluster_i.push_back(cur_point_ix);
			nt = tree->search(points[cur_point_ix], distanceTol);
			std::copy(nt.begin(), nt.end(), std::inserter(neighbours, neighbours.end()));
		}
		clusters.push_back(cluster_i);
		cluster_i.clear();
	}
	return clusters;
}
