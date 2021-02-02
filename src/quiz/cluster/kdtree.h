/* \author Aaron Brown */
// Quiz on implementing kd tree

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
        int x_y = (depth%2);
        Node** next_node = (point[x_y] < (*node)->point[x_y])? &((*node)->left): &((*node)->right);
        InsertHelper(next_node, point, id, depth+1);
      }
  };

	void insert(std::vector<float> point, int id)
	{
    InsertHelper(&root, point, id, 0);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




