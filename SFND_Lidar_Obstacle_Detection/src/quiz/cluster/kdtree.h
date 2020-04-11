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

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if (root == NULL){
			root = new Node(point, id);
			return;
		}

		Node* curr = root;
		Node* temp = NULL;
		int child;
		int depth = 0;
		while (curr != NULL){
			int dim = depth % 2;
			temp = curr;
			if (point[dim] <= curr->point[dim]){
				curr = curr->left;
				child = 0;
			}
			else{
				curr = curr->right;
				child = 1;
			}
			depth += 1;
		}
		curr = new Node(point, id);
		if (child == 0){
			temp->left = curr;
		}
		else{
			temp->right = curr;
		}
	}

	void iterSearch(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int> &ids){
		if (node == NULL)
			return;

		float x = target[0];
		float y = target[1];
		float nx = node->point[0];
		float ny = node->point[1];
		if ((nx >= x - distanceTol) && (nx <= x + distanceTol) && 
		    (ny >= y - distanceTol) && (ny <= y + distanceTol)){
			
			float dist = (x-nx)*(x-nx) + (y-ny)*(y-ny);
			dist = sqrt(dist);
			if (dist <= distanceTol){
				ids.push_back(node->id);
			}
		}

		int dim = depth % 2;
		if (target[dim]-distanceTol < node->point[dim]){
			iterSearch(target, node->left, depth+1, distanceTol, ids);
		}
		if (target[dim]+distanceTol > node->point[dim]){
			iterSearch(target, node->right, depth+1, distanceTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		iterSearch(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




