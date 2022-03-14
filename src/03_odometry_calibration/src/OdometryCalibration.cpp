#include <odometry_calibration/OdometryCalibration.h>
#include <cmath>
#include <iostream>

namespace odometry_calibration {

/**
 * \brief Computes the odometry error function.
 * \param[in] groundTruth The ground truth odometry measured by an external sensor (called u* in the slides).
 * \param[in] observation The odometry measurement observed by the robot itself, e.g., using wheel sensors or joint angles (called u in the slides).
 * \param[in] calibrationMatrix The calibration matrix of the current iteration (called X in the slides).
 * \return The error function vector (called e in the slides).
 */
Eigen::Vector3d OdometryCalibration::errorFunction(const Odometry& groundTruth, const Odometry& observation, const Eigen::Matrix3d& calibrationMatrix) {
	Eigen::Vector3d error;
	Eigen::Vector3d gT(groundTruth.ux, groundTruth.uy, groundTruth.utheta);
	Eigen::Vector3d o(observation.ux, observation.uy, observation.utheta);
	error = gT - calibrationMatrix*o;
	return error;
}

/**
 * \brief Computes the Jacobian (matrix derivative) of the error function for a given odometry measurement.
 * \param[in] measurement The odometry measured by the robot (called u in the slides).
 * \return The Jacobian matrix of the error function.
 */
Eigen::Matrix3Xd OdometryCalibration::jacobian(const Odometry& observation) {
	Eigen::Matrix3Xd jacobian(3, 9);
	jacobian << -observation.ux, -observation.uy, -observation.utheta, 0, 0, 0, 0, 0, 0,
				0, 0, 0, -observation.ux, -observation.uy, -observation.utheta, 0, 0, 0,
				0, 0, 0, 0, 0, 0, -observation.ux, -observation.uy, -observation.utheta;
	return jacobian;
}

/**
 * \brief Calibrates the odometry of a robot.
 * \param[in] measurementData A vector containing ground truth and observed odometry measurements.
 * ŗeturn The calibration matrix that can be used to correct the odometry.
 */
Eigen::Matrix3d OdometryCalibration::calibrateOdometry(const std::vector<MeasurementData>& measurements) {
	Eigen::Matrix3d calibrationMatrix = Eigen::Matrix3d::Identity();

	std::vector<Eigen::Vector3d> e;
	std::vector<Eigen::Matrix3Xd> J;
	Eigen::VectorXd neg_b = Eigen::VectorXd::Zero(9);
	Eigen::MatrixXd Omega = Eigen::MatrixXd::Identity(9, 9);
	Eigen::MatrixXd H = Eigen::MatrixXd::Zero(9, 9);

	for(int i=0; i<measurements.size(); i++)
	{
		e.push_back(errorFunction(measurements[i].groundTruth, measurements[i].uncalibrated, calibrationMatrix));
		J.push_back(jacobian(measurements[i].uncalibrated));
	}

	for(int i=0; i<measurements.size(); i++)
	{
		neg_b -= e[i].transpose()*J[i];
		H += J[i].transpose()*J[i];
	}

	Eigen::VectorXd add(9);
	add = H.colPivHouseholderQr().solve(neg_b);

	calibrationMatrix += Eigen::Map<Eigen::Matrix3d>(add.data()).transpose();

	return calibrationMatrix;
}

/**
 * \brief Applies the calibration matrix to an odometry measurement in order to get a corrected estimate.
 * \param[in] uncalibratedOdometry An uncalibrated odometry measurement (called u in the slides).
 * \param[in] calibrationMatrix The calibration matrix computed by calibrateOdometry in a previous step (called X in the slides).
 * \return The corrected odometry estimate.
 */
Odometry OdometryCalibration::applyOdometryCorrection(const Odometry& uncalibratedOdometry, const Eigen::Matrix3d& calibrationMatrix) {
	Odometry calibratedOdometry;
	
	Eigen::Vector3d uO(uncalibratedOdometry.ux, uncalibratedOdometry.uy, uncalibratedOdometry.utheta);
	Eigen::Vector3d res = calibrationMatrix*uO;
	calibratedOdometry.ux = res(0);
	calibratedOdometry.uy = res(1);
	calibratedOdometry.utheta = res(2);

	return calibratedOdometry;
}

/**
 * \brief Converts an odometry reading into an affine 2D transformation.
 * \param[in] odometry The odometry reading.
 * \returns The corresponding affine transformation as a 3x3 matrix.
 */
Eigen::Matrix3d OdometryCalibration::odometryToAffineTransformation(const Odometry& odometry) {
	Eigen::Matrix3d transformation;
	transformation << 	cos(odometry.utheta), -sin(odometry.utheta), odometry.ux,
						sin(odometry.utheta), cos(odometry.utheta), odometry.uy,
						0, 0, 1;
	return transformation;
}

/**
 * \brief Converts an affine 2D transformation matrix into a 2D robot pose (x, y, and theta).
 * \param[in] transformation An affine transformation as a 3x3 matrix.
 * \returns The corresponding 2D pose (x, y, and theta).
 */
Pose2D OdometryCalibration::affineTransformationToPose(const Eigen::Matrix3d& transformation) {
	Pose2D pose;
	/* TODO: replace the following lines by the x and y position and the rotation of the robot.
	 * Hint: x and y can be directly read from the matrix. To get the rotation, use the acos/asin
	 * functions on the rotation matrix and take extra care of the sign, or (better) use the
	 * atan2 function.
	 */
	pose.x = transformation(0, 2);
	pose.y = transformation(1, 2);
	pose.theta = atan2(transformation(1, 0), transformation(0, 0));
	return pose;
}

/**
 * \brief Calculate the robot's trajectory in Cartesian coordinates given a list of calibrated odometry measurements.
 * \param[in] calibratedOdometry Odometry measurements that have already been corrected using the applyOdometryCorrection method.
 * \returns A vector of 2D poses in the global coordinate system.
 */
std::vector<Pose2D> OdometryCalibration::calculateTrajectory(const std::vector<Odometry>& calibratedOdometry) {
	std::vector<Pose2D> trajectory;

	Pose2D pose;
	Eigen::Matrix3d new_transformation;
	Eigen::Matrix3d sum_transformation = Eigen::Matrix3d::Identity();



	for(int i=0; i<calibratedOdometry.size(); i++)
	{
		new_transformation = odometryToAffineTransformation(calibratedOdometry[i]);
		sum_transformation *= new_transformation;
		pose = affineTransformationToPose(sum_transformation);
		trajectory.push_back(pose);
	}

	return trajectory;
}




} /* namespace odometry_calibration */
