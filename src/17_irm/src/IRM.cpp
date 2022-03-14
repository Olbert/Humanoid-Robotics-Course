#include <irm/IRM.h>
#include <angles/angles.h>
#include <vector>

namespace irm {

	/**
	 * \brief Draws an angle for each joint randomly from a uniform distribution.
	 * \return The vector (q0, q1, q2) of random joint angles.
	 */
	Eigen::Vector3d IRM::sampleConfiguration() const {
		Eigen::Vector3d randomJointAngles = Eigen::Vector3d::Zero();

		// TODO Sample the joint angles.

		// CAUTION: Do not call srand() in this method. The random generator is already seeded.
		randomJointAngles[0] = (M_PI / 2 - 0) * ((double)rand() / (double)RAND_MAX) + 0; //0 + rand() % (M_PI / 2); 
		randomJointAngles[1] = (M_PI + M_PI) * ((double)rand() / (double)RAND_MAX) - M_PI; //-M_PI + rand() % M_PI;
		randomJointAngles[2] = (M_PI + M_PI) * ((double)rand() / (double)RAND_MAX) - M_PI; //-M_PI + rand() % M_PI;

		return randomJointAngles;
	}

	/**
	 * \brief Computes the manipulability of a robot configuration.
	 * \param[in] jointAngles The angles of each joint (q_0, q_1, q_2).
	 * \param[in] endeffectorPose The endeffector pose (e_x, e_y, e_theta).
	 * \return The manipulability score between 0.0 (bad pose) and 1.0 (good pose).
	 */
	double IRM::computeManipulability(const Eigen::Vector3d & jointAngles, const Eigen::Vector3d & endeffectorPose) const {
		double manipulability = 0.0;
		// TODO Compute the manipulability score according to the function on the exercise sheet.
		manipulability = 1 - 1 / (4 * M_PI) * (abs(4 * jointAngles[0] - M_PI) + abs(jointAngles[1]) + abs(jointAngles[2]) + abs(endeffectorPose[2]));
		return manipulability;
	}

	/**
	 * \brief Computes the reachability map of the robot.
	 * \param[in] numSamples The number of samples that the method should add to the reachability map.
	 */
	void IRM::computeRM(const size_t & numSamples) {
		// TODO: Compute the reachability map.

		/* Available methods:
		 *
		 * - sampleConfiguration(): defined above, draws a random joint configuration
		 *
		 * - computeManipulability(): defined above, computes the manipulability score.
		 *
		 * - addToRM(const Eigen::Vector3d& jointAngles, const Eigen::Vector3d& endeffectorPose, const double& manipulability):
		 *       Stores the given values to the reachability map.
		 *
		 * - bool forwardKinematics(const Eigen::Vector3d& jointAngles, Eigen::Vector3d& endeffectorPose):
		 *       Calculates the forward kinematics for jointAngles and stores the result in endeffectorPose.
		 *       The method returns true on success and false on failure (e.g., if the joint angles are
		 *       beyond the robot's limits).
		 *
		 * - bool inverseKinematics(const Eigen::Vector3d& endeffectorPose, Eigen::Vector3d& jointAngles):
		 *       Calculates the inverse kinematics for an endeffectorPose and stores the result in jointAngles.
		 *       The method returns true on success and false on failure (e.g., if the endeffector pose is
		 *       impossible to reach).
		 *
		 */
		for (int i = 0; i < numSamples; i++)
		{
			Eigen::Vector3d endeffectorPose = Eigen::Vector3d();
			Eigen::Vector3d jointAngles = Eigen::Vector3d();
			endeffectorPose[1] = 0;

			while (endeffectorPose[1] <= 0)
			{
				jointAngles = sampleConfiguration();
				forwardKinematics(jointAngles, endeffectorPose);
			}
			double manipulability = computeManipulability(jointAngles, endeffectorPose);

			addToRM(jointAngles, endeffectorPose, manipulability);
		}
	}

	/**
	 * \brief Computes the inverse reachability map.
	 * \param[in] voxels The voxels of the reachability map computed by the method above.
	 */
	void IRM::computeIRM(const std::vector<RMVoxel> & voxels) {
		// TODO: Compute the inverse reachability map.

		/* Available methods and fields:
		 * - voxels[i].configurations: a standard vector containing the configurations of the voxel.
		 *
		 * - voxels[i].configurations[j]->jointAngles: The joint angles of the given configuration.
		 * - voxels[i].configurations[j]->manipulability: The manipulability score of the given configuration.
		 *
		 * - addToIRM(const Eigen::Vector3d& basePose, const Eigen::Vector3d& jointAngles, const double& manipulability):
		 *       Stores the given values to the reachability map.
		 *
		 * - bool forwardKinematics(const Eigen::Vector3d& jointAngles, Eigen::Vector3d& endeffectorPose):
		 *       Calculates the forward kinematics for jointAngles and stores the result in endeffectorPose.
		 *       The method returns true on success and false on failure (e.g., if the joint angles are
		 *       beyond the robot's limits).
		 *
		 * - bool inverseKinematics(const Eigen::Vector3d& endeffectorPose, Eigen::Vector3d& jointAngles):
		 *       Calculates the inverse kinematics for an endeffectorPose and stores the result in jointAngles.
		 *       The method returns true on success and false on failure (e.g., if the endeffector pose is
		 *       impossible to reach).
		 *
		 */
		/*
		for (int i = 0; i < voxels.size(); i++)
		{
			Eigen::Vector3d& endeffectorPose = Eigen::Vector3d();
			forwardKinematics(voxels[i].configurations[0]->jointAngles, endeffectorPose);
			Eigen::Vector3d jointAngles = voxels[i].configurations[0]->jointAngles;
			inverseKinematics(endeffectorPose, jointAngles);
			/*
			Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();

			mat <<	cos(angle), -sin(angle),0, x
					sin(angle),	cos(angle), 0, y
					0,			0,			1; z
					0,			0,			0, 1;
			
			addToRM(jointAngles, endeffectorPose, voxels[i].configurations[0]->manipulability);
		}
		*/
	}


}  // namespace irm
