#include <octree/Octree.h>
#include <iostream>

namespace octree {

/**
 * \brief Inserts a new point into the octree and marks the corresponding cell as occupied.
 * \param[in] point The 3D point to insert into the octree.
 * \return true if the point has been successfully inserted, false otherwise
 *
 * This method implements the framework for your application. You don't have to
 * change anything here.
 */
bool Octree::insertPoint(const Eigen::Vector3d& point) {
	// Find existing node that contains the point
	Node *node = findNode(point);
	if (!node) {
		// This should not happen.
		std::cerr << "Could not find node containing the new point." << std::endl;
		return false;
	}

	// Is the node already marked as occupied? Then there's nothing do do.
	if (node->content == OCCUPIED) {
		return true;
	}

	// Is the node already at maximum depth? Then mark it as occupied
	if (node->depth == maxDepth) {
		node->content = OCCUPIED;
	} else {
		// Split the node recursively until the maximum depth is reached
		while (node->depth < maxDepth) {
			node = node->split(point);
			if (!node) {
				// This should not happen.
				std::cerr << "Splitting node returned NULL." << std::endl;
				return false;
			}
		}
	}

	// Traverse the tree towards the root and merge cells
	for (node = node->parent; node != NULL; node = node->parent) {
		const bool merged = node->merge();
		if (!merged) {
			// Could not merge -> nothing to do anymore
			break;
		}
	}
	return true;
}

/**
 * \brief Returns the index (0-7) of the child cube containing a given 3D point.
 * \param[in] point The 3D point
 * \return The index of the child cube containing the point.
 */
unsigned int Node::findIndex(const Eigen::Vector3d& point) const {
	unsigned int index = 0;
	/* TODO: Compute the index of the child node containing the given point.
	 *
	 * Member variables that you can use:
	 * - const Eigen::Vector3d corner1: first corner of the current cube
	 * - const Eigen::Vector3d corner2: opposite corner of the current cube
	 */

	if(point(0) > (corner2(0)+corner1(0))/2)
		index += 1;
	if(point(1) > (corner2(1)+corner1(1))/2)
		index += 2;
	if(point(2) > (corner2(2)+corner1(2))/2)
		index += 4;

	return index;
}

/**
 * \brief Traverses the tree from the root to find existing node that contains a given point.
 * \param[in] point The 3D point
 * \return Node containing the given 3D point
 */
Node* Octree::findNode(const Eigen::Vector3d& point) const {
	Node *result = NULL;

	/* TODO: Find and return the leaf node containing the given point.
	 *
	 * Available member variables and methods:
	 * - Node * const root: the root node of the tree
	 * - node->children[8]: the 8 children of the node (all NULL if node is a leaf)
	 * - node->findIndex(const Eigen::Vector3d& point): method defined above
	 */

	Node *node = root;
	unsigned int childIndex;

	while(result == NULL)
	{
		childIndex = node->findIndex(point);
		if(node->children[childIndex] != NULL)
			node = node->children[childIndex];
		else
			result = node;
	}

	return result;
}

/**
 * \brief Splits the current node into 8 child nodes and returns the child node containing the given point.
 * \param[in] point The 3D point.
 * \return The newly created child node containing the given point.
 */
Node* Node::split(const Eigen::Vector3d& point) {
	Node *result = NULL;

	/* TODO:
	 * 1. Set the current node's content to MIXED
	 * 2. Create 8 child nodes for children[0], ..., children[8].
	 *    The Node constructor has the signature
	 *    Node(Eigen::Vector3d corner1,         // = first corner
	 *         Eigen::Vector3d corner2,         // = opposite corner
	 *         Node *parent,                    // = this
	 *         unsigned int depth,              // = depth of the child node within the tree
	 *         Content content)                 // = FREE, OCCUPIED, or MIXED
	 * 3. Return the child node containing the point.
	 *
	 * Available member variables and methods:
	 * - Content content: label of the current node (FREE, OCCUPIED, or MIXED)
	 * - const unsigned int depth: depth of the current node within the tree (0 = root node)
	 * - const Eigen::Vector3d corner1: first corner of the current cube
	 * - const Eigen::Vector3d corner2: opposite corner of the current cube
	 * - Node * children[8]: child node pointers
	 * - Node * const parent: pointer to the parent node (NULL for the root node)
	 * - findIndex(const Eigen::Vector3d& point): method defined above
	 * - findNode(const Eigen::Vector3d& point): method defined above
	 */

	double side = (corner2(0) - corner1(0))/2;

	Eigen::Vector3d cor1;
	Eigen::Vector3d cor2;
	content = MIXED;
	for(int i=0; i<8; i++)
	{
		int j = i;
		if(j%2 == 0)
		{
			cor1(0) = corner1(0);
			cor2(0) = corner1(0) + side;
		}
		else
		{
			cor1(0) = corner2(0) - side;
			cor2(0) = corner2(0);
		}
		j /= 2;

		if(j%2 == 0)
		{
			cor1(1) = corner1(1);
			cor2(1) = corner1(1) + side;
		}
		else
		{
			cor1(1) = corner2(1) - side;
			cor2(1) = corner2(1);
		}
		j /= 2;

		if(j%2 == 0)
		{
			cor1(2) = corner1(2);
			cor2(2) = corner1(2) + side;
		}
		else
		{
			cor1(2) = corner2(2) - side;
			cor2(2) = corner2(2);
		}

		children[i] = new Node(cor1, cor2, this, depth+1, FREE);
	}

	result = children[findIndex(point)];
	result->content = OCCUPIED;

	return result;
}

/**
 * \brief Tries to merge the current node in case all children have the same content.
 * \return True if the children have been merged, false otherwise.
 */
bool Node::merge() {
	bool merged = false;

	/* TODO:
	 * 1. Check if the children can be merged
	 * 2. Set the label of the current node
	 * 3. Delete all children with "delete children[i]"
	 * 4. Set all children to NULL
	 *
	 * Available member variables and methods:
	 * - Content content: label of the current node (FREE, OCCUPIED, or MIXED)
	 * - Node * children[8]: child node pointers
	 * - const unsigned int depth: depth of the current node within the tree (0 = root node)
	 */

	Content childrenContent = children[0]->content;

	if(childrenContent == MIXED)
	{
		merged = false;
		return merged;
	}

	for(int i=1; i<8; i++)
		if(children[i]->content != childrenContent)
		{
			merged = false;
			return merged;
		}

	content = childrenContent;

	for(int i=0; i<8; i++)
	{
		delete children[i];
		children[i] = NULL;
	}

	merged = true;
	return merged;
}

}  // namespace octree
