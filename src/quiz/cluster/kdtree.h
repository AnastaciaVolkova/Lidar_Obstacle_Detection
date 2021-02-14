#ifndef KDTREE_H_
#define KDTREE_H_
#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

  void InsertHelper(Node** node, std::vector<float>& point, int id, int depth){
      if (*node == NULL){
        *node = new Node(point, id);
      } else {
        int x_y = (depth%point.size());
        Node** next_node = (point[x_y] < (*node)->point[x_y])? &((*node)->left): &((*node)->right);
        InsertHelper(next_node, point, id, depth+1);
      }
  };

	void insert(std::vector<float> point, int id)
	{
		InsertHelper(&root, point, id, 0);
	};

	void SearchHelper(Node* node, int depth, float distanceTol, std::vector<float>& target, std::vector<int>& ids){
		if (node == NULL)
			return;

		// Check if current point is inside target box.
		if (abs(node->point[0]-target[0])<distanceTol && abs(node->point[1]-target[1])<distanceTol) {
			float distance = 0;
			for (int i = 0; i < target.size(); i++)
				distance += pow(node->point[i] - target[i], 2);
			distance = sqrt(distance);
			if (distance <= distanceTol)
				ids.push_back(node->id);
		}

		int x_y = depth%target.size();

		if (target[x_y]-distanceTol < node->point[x_y])
			SearchHelper(node->left, depth+1, distanceTol, target, ids);
		if (target[x_y]+distanceTol > node->point[x_y])
			SearchHelper(node->right, depth+1, distanceTol, target, ids);
	};

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		SearchHelper(root, 0, distanceTol, target, ids);
		return ids;
	}
};
#endif
