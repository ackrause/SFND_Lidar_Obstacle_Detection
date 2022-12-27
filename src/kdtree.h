/* \author Andrew Krause */

#ifndef KDTREE_H_
#define KDTREE_H_

#include <cmath>
#include <vector>

// Structure to represent node of kd tree
template<typename PointT>
struct Node {
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree {
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node<PointT>*& currNode, uint depth, PointT point, int id)
	{
		if (currNode == NULL){
			currNode = new Node<PointT>(point, id);
		} else {
            // compare coordinates based on what level of tree we are on, rotating between x, y, and z
            switch(depth % 3){
                case 0:
                    if (point.x < currNode->point.x) {
                        insertHelper(currNode->left, depth+1, point, id);
                    } else {
                        insertHelper(currNode->right, depth+1, point, id);
                    }
                    break;
                case 1:
                    if (point.y < currNode->point.y) {
                        insertHelper(currNode->left, depth+1, point, id);
                    } else {
                        insertHelper(currNode->right, depth+1, point, id);
                    }
                    break;
                case 2:
                    if (point.z < currNode->point.z) {
                        insertHelper(currNode->left, depth+1, point, id);
                    } else {
                        insertHelper(currNode->right, depth+1, point, id);
                    }
                    break;
            }
        }
	}

	void insert(PointT point, int id)
	{
		insertHelper(root, 0, point, id);
	}

	void searchHelper(std::vector<int> &ids, Node<PointT>* currNode, PointT target, float distanceTol, uint depth)
	{
        // if we reached the end of the tree, return
		if (currNode == NULL) {
			return;
		}

        float deltaX = currNode->point.x - target.x;
        float deltaY = currNode->point.y - target.y;
        float deltaZ = currNode->point.z - target.z;

        // if currentNode is within distanceTol of target, add it to our cluster
        if (std::abs(deltaX) <= distanceTol &&
            std::abs(deltaY) <= distanceTol &&
            std::abs(deltaZ) <= distanceTol &&
            std::sqrt(deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ) <= distanceTol) {
            ids.push_back(currNode->id);
        }

        // check the rest of the tree
        switch(depth % 3) {
            case 0:
                if (deltaX > -distanceTol) {
                    searchHelper(ids, currNode->left, target, distanceTol, depth+1);
                }
                if (deltaX < distanceTol) {
                    searchHelper(ids, currNode->right, target, distanceTol, depth+1);
                }
                break;
            case 1:
                if (deltaY > -distanceTol) {
                    searchHelper(ids, currNode->left, target, distanceTol, depth+1);
                }
                if (deltaY < distanceTol) {
                    searchHelper(ids, currNode->right, target, distanceTol, depth+1);
                }
                break;
            case 2:
                if (deltaZ > -distanceTol) {
                    searchHelper(ids, currNode->left, target, distanceTol, depth+1);
                }
                if (deltaZ < distanceTol) {
                    searchHelper(ids, currNode->right, target, distanceTol, depth+1);
                }
                break;
        }
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(ids, root, target, distanceTol, 0);

		return ids;
	}
};
#endif /* KDTREE_H_ */
