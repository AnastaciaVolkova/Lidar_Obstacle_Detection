#ifndef CLUSTER_H_
#define CLUSTER_H_
#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"
#include <set>

// Function returns list of indices for each cluster
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);
#endif