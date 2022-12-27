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

	void insertHelper(Node *newNode, Node *&currNode, uint depth)
	{
		if (currNode == NULL)
		{
			currNode = newNode;
		}
		else if (newNode->point[depth % 2] < currNode->point[depth % 2])
		{
			insertHelper(newNode, currNode->left, depth+1);
		}
		else
		{
			insertHelper(newNode, currNode->right, depth+1);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		Node *newNode = new Node(point, id);

		insertHelper(newNode, root, 0);
	}

	void searchHelper(std::vector<int> &ids, Node *currNode, std::vector<float> target, float distanceTol, uint depth)
	{
		if (currNode == NULL)
		{
			return;
		}
		// if currentNode is within distanceTol of target 
		else if (currNode->point[0] <= (target[0] + distanceTol) &&
				 currNode->point[0] >= (target[0] - distanceTol) &&
				 currNode->point[1] <= (target[1] + distanceTol) &&
				 currNode->point[1] >= (target[1] - distanceTol) &&
				 std::sqrt(std::pow(target[0] - currNode->point[0], 2.0) + std::pow(target[1] - currNode->point[1], 2.0)) <= distanceTol)
		{
			ids.push_back(currNode->id);

			searchHelper(ids, currNode->left, target, distanceTol, depth+1);
			searchHelper(ids, currNode->right, target, distanceTol, depth+1);
		}
		else // currentNode is outside of distanceTol of target
		{
			if (target[depth%2] - distanceTol <= currNode->point[depth % 2])
			{
				searchHelper(ids, currNode->left, target, distanceTol, depth+1);
			}
			
			if (target[depth%2] + distanceTol >= currNode->point[depth%2])
			{
				searchHelper(ids, currNode->right, target, distanceTol, depth+1);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(ids, root, target, distanceTol, 0);

		return ids;
	}
};
