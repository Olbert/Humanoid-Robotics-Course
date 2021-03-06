#include <ara_star/ARAStar.h>
#include <ctime>
#include <cmath>

namespace ara_star {

/**
 * @brief ARA* heuristic.
 * @param currentNode The current grid cell.
 * @param goalNode The goal grid cell.
 * @return The heuristic value.
 */
double ARAStarHeuristic::heuristic(
		const GridNode * const currentNode,
		const GridNode * const goalNode) const
{
	double result = 0.0;
	// TODO
	/* Available fields and methods:
	 * - node->x: the x index of the cell
	 * - node->y: the y index of the cell
	 * - getW(): the current w value of ARA*
	 */

	double dist = sqrt((currentNode->x - goalNode->x)*(currentNode->x - goalNode->x) + (currentNode->y - goalNode->y)*(currentNode->y - goalNode->y));

	result = getW()*dist;

	return result;
}

/**
 * @brief Plans a path from a start node to a goal node while checking that the planning does not exceed the time limit.
 * @param[in] startNode The start node.
 * @param[in] goalNode The goal node.
 * @param[in] tstart The start time at which the ARA* was initially executed.
 * @param[in] timeLimit The time limit in seconds, which should not be exceeded.
 * @return The optimal path from the start node to the end node.
 */
std::deque<const AbstractNode*> ARAStarPlanning::planPath(
		const AbstractNode * const startNode,
		const AbstractNode * const goalNode,
		const time_t & tstart,
		const double& timeLimit)
{
	// Create empty lists for open nodes and closed nodes
   	OpenList openList;
   	ClosedList closedList;

   	std::deque<const AbstractNode*> resultPath;

   	/* TODO: Fill resultPath by planning a path from the startNode to the goalNode */

   	/*
	**** Hint: Copy your code that you have implemented for exercise 11
	**** (PathPlanning::planPath method) and extend it so that it aborts
	**** searching for a path as soon as the time limit is exceeded.
	*/

   	/* Available methods: all of the methods available in PathPlanning::planPath (exercise 11) plus:
   	 * - difftime(time(0), tstart): returns the time elapsed since tstart in seconds.
   	 */


   	// Create empty lists for open nodes and closed nodes

	openList.enqueue(startNode, 0.0);
	while (!openList.isEmpty()) {
		if (difftime(time(0), tstart) >= timeLimit)
		{
			resultPath.clear();
			break;
		}
		const AbstractNode * currentNode = openList.removeMin();
		if (isCloseToGoal(currentNode, goalNode)) {
			resultPath = followPath(currentNode);
			break;
		}
		closedList.add(currentNode);
		expandNode(currentNode, goalNode, openList, closedList);
	}


	return resultPath;
}

/**
 * @brief Runs the ARA* algorithm
 * @param wInitial The initial value for w that should be used in the first iteration.
 * @param wDelta Step with for decreasing w in each iteration (wDelta is > 0).
 * @param timeLimit Time limit in seconds that should not be exceeded.
 * @param startNode The start node where the robot currently is located.
 * @param goalNode The goal node that the robot should reach.
 * @return The path from the start node to the goal node (empty if no path was found).
 */
std::deque<const AbstractNode*>  ARAStarPlanning::runARA(	const double& wInitial, const double& wDelta, const double& timeLimit,
															const ara_star::AbstractNode * const startNode,
															const ara_star::AbstractNode * const goalNode)
{
	std::deque<const AbstractNode*> resultPath;
	std::deque<const AbstractNode*> temp;

	const time_t tstart = time(0);

	/*
	 * TODO: Implement the ARA* algorithm.
	 */

	/* Available methods:
	 * - heuristic_.setW(): Sets the current w value for the ARA* heuristic.
	 * - heuristic_.getW(): Returns the current w value set using setW().
	 * - planPath(startNode, goalNode, tstart, timeLimit): Runs the planning method that you implemented above.
	 * - difftime(time(0), tstart): returns the ellapsed time in seconds.
	 */

	heuristic_.setW(wInitial);

	while (heuristic_.getW() >= 1. && difftime(time(0), tstart) < timeLimit)
	{
		temp = planPath(startNode, goalNode, tstart, timeLimit);
		std::cout<<heuristic_.getW()<<": "<<temp.size()<<std::endl;
		if (temp.size() == 0)
			break;
		resultPath = temp;

		heuristic_.setW(heuristic_.getW() - wDelta);
	}

	return resultPath;


}

}
