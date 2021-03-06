#include <icp/ICP.h>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <math.h>
#include <Eigen/SVD> 
#include <limits>

using namespace std;
namespace icp
{
/**
 * \brief Compute the Euclidean distance between a pair of 2D points.
 * \param[in] p1: The first 2D point.
 * \param[in] p2: The second 2D point.
 * \return The Euclidean distance between the two input 2D points.
 */
	double ICP::distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
	{
		double result = -1.0;
		//TODO: Calculate the distance between the points.

		result = sqrt((p2(0)-p1(0))*(p2(0)-p1(0)) + (p2(1)-p1(1))*(p2(1)-p1(1)));

		return result;
	}
/**
 * \brief Compute the closest point that lies on a given line to a given 2D point.
 * \param[in] pX: The given 2D point, to which we need to compute the closest point that lies on a line.
 * \param[in] pL1: A point that lies on the line.
 * \param[in] pL2: Another point that lies on the line.
 * \return The closest point on the line.
 */
	Eigen::Vector2d ICP::closestPointOnLine(const Eigen::Vector2d& pX, const Eigen::Vector2d& pL1, const Eigen::Vector2d& pL2)
	{
		Eigen::Vector2d result(0., 0.);
		//TODO: Compute the point on the line (pL1-pL2) that is closest to pX.

		Eigen::Vector2d pL12 = pL2 - pL1;
		Eigen::Vector2d pXL1 = pX - pL1;

		float lengthSqrpL12 = pL12(0) * pL12(0) + pL12(1) * pL12(1);
		float t = (pXL1(0) * pL12(0) + pXL1(1) * pL12(1)) / lengthSqrpL12;

		result = pL1 + t * pL12;

		return result;


	}
/**
 * \brief Get the minimum value within vector.
 * \param[in] dist: A vector of values.
 * \return The minimum value.
 */
	double ICP::minDistance(const std::vector<double>& dist)
	{
		double result = 0.0;
		//TODO: Find and return the minimum value in the vector dist

		result = std::numeric_limits<double>::infinity();

		for(int i=0; i<dist.size(); i++)
			if(dist[i] < result)
				result = dist[i];

		return result;
	}
/**
 * \brief Compute the corresponding points in list P to those points in list Q, using the 'closest point' matching method.
 * \param[in] Q: A vector of 2D points.
 * \param[in] P: A vector of 2D points.
 * \return A vector of the corresponding 2D points matched to points of list Q in list P.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 * The result vector will have the same length as Q.
 */
	StdVectorOfVector2d ICP::euclideanCorrespondences(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& P)
	{
		StdVectorOfVector2d result;
		//TODO: Compute corresponding points using the "closest point" method

		return result;
	}
/**
 * \brief Compute the corresponding points in list P to those points in list Q, using the 'point-to-line' matching method .
 * \param[in] Q: A vector of 2D points.
 * \param[in] P: A vector of 2D points.
 * \return A vector of the corresponding 2D points matched to points of list Q in list P.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	StdVectorOfVector2d ICP::closestPointToLineCorrespondences(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& P)
	{
		StdVectorOfVector2d  result;
		//TODO: Compute corresponding points using the "point-to-line" method

		return result;

	}
/**
 * \brief Compute the affine transformation matrix needed to align the previously computed corresponding points (list C) to the points of list Q.
 * \param[in] Q: A vector of 2D points.
 * \param[in] C: A vector of 2D points, that corresponds to points in list Q.
 * \return An Affine transformation matrix.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	Eigen::Matrix3d ICP::calculateAffineTransformation(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& C)
	{
		Eigen::Matrix3d result = Eigen::Matrix3d::Zero();
		//TODO: Compute the affine transformation matrix


		return result;

	}
/**
 * \brief Apply the affine transformation matrix on the points in list P.
 * \param[in] A: Affine transformation matrix.
 * \param[in] P: A vector of 2D points, on which the affine transformation will be applied.
 * \return The vector of transformed points.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	StdVectorOfVector2d ICP::applyTransformation(const Eigen::Matrix3d& A, const StdVectorOfVector2d& P)
	{
		StdVectorOfVector2d  result;

		//TODO: Apply the affine transformation A to the points in list P.

		return result;
	}
/**
 * \brief Compute the error between the points in Q list and the transformed corresponding points.
 * \param[in] Q: A vector of 2D points.
 * \param[in] C: A vector of 2D points corresponding to point in Q.
 * \param[in] A: Affine transformation matrix.
 * \return The error of applying the transformation.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	double ICP::computeError(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& C, const Eigen::Matrix3d& A)
	{
		double result = -1.0;
		//TODO: Compute the error after the transformation.
		return result;		
	}


/**
 * \brief Perform one iteration of ICP and prints the error of that iteration.
 * \param[in] Q: A vector of 2D points.
 * \param[in] P: A vector of 2D points, that we need to transform them to be aligned with points in Q list.
 * \param[out] convergenceFlag: A flag passed by reference to determine whether the alignment error has crossed the convergence threshold or not. The flag should be set to 'true' in case of convergence.
 * \param[in] pointToLineFlag: A flag that states the matching method to be used. Its value is set to 'true' when point-to-line method is needed, and to 'false' when closest point method is needed.
 * \param[in] threshold: The maximum value of acceptable error.
 * \return The vector of transformed points.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	StdVectorOfVector2d ICP::iterateOnce(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& P, bool & convergenceFlag, const bool pointToLineFlag, const double threshold)
	{
		StdVectorOfVector2d result;
		
		//TODO: Perform one iteration of ICP
	
		double error = 0.0;
		//TODO: Compute the error

		if (error <= threshold) {
			convergenceFlag = true;
		}

		return result;
	}


}
