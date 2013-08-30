#ifndef CROBOTPLATFORMDIFFERENTIALDRIVE_H_
#define CROBOTPLATFORMDIFFERENTIALDRIVE_H_

#include "interface_robot_platform.h"
#include <iostream>

#define ROBOT_DIFFERENTIAL_DRIVE_ODOMETRY_EPSILON 0.001

/**
 * @struct StrDifferentialDriveRobotProperties
 * @brief General properties of a differential robot
 * @author Dirk Holz, Fraunhofer IAIS, August 2007
 */
struct StrDifferentialDriveRobotProperties
{
	/** Constructor to initialize values. The value '0' is not used to avoid division by zero. */
	StrDifferentialDriveRobotProperties()
	{
		dAxisLength = 1;
		dWheelCircumferenceLeft = 1;
		dWheelCircumferenceRight = 1;
		unTicsPerTurnOfWheelLeft = 1;
		unTicsPerTurnOfWheelRight = 1;
		dTurningAdaptation = 1;
		bLeftMotorInverted = false;
		bRightMotorInverted = false;
	}
	;
	double dAxisLength;
	double dWheelCircumferenceLeft;
	double dWheelCircumferenceRight;
	unsigned int unTicsPerTurnOfWheelLeft;
	unsigned int unTicsPerTurnOfWheelRight;
	bool bLeftMotorInverted;
	bool bRightMotorInverted;

	double dTurningAdaptation;
};

/**
 * @class CDifferentialDriveRobotPlatform
 * @brief Abstract Implementation of a differential drive robot
 * @author Dirk Holz, Fraunhofer IAIS, August 2007
 */
class CRobotPlatformDifferentialDrive : public IRobotPlatform
{
 public:
	CRobotPlatformDifferentialDrive();
	virtual ~CRobotPlatformDifferentialDrive();

	/**
	 * Set translational and angular velocities of the wheels.
	 * The specified velocities are used to compute the translational velocities
	 * of left and right wheels for a differential drive robot.
	 * @param dTranslationalVelocity Translational Velocity to set.
	 * @param dAngularVelocity Angular Velocity to set.
	 * @return void
	 */
	void move(double dTranslationalVelocity, double dAngularVelocity);

	/**
	 * Set translational velocities of the wheels (separately). The actual implementation
	 * depends on the type of the chosen robot.
	 * @param dSpeedLeft Speed of left wheel(s).
	 * @param dSpeedRight Speed of right wheel(s).
	 * @return void
	 */
	virtual void setSpeed(double dSpeedLeft, double dSpeedRight) = 0;

	/**
	 * Set translational velocities relative to the maximum speed of the robot,
	 * i.e. a value of 0 will stop the robot, a value of 1 will move the robot
	 * with maximum speed.
	 * @param dSpeedLeft Relative speed of left wheel(s).
	 * @param dSpeedLeft Relative speed of right wheel(s).
	 * @return void
	 */
	void setSpeedFactors(double dSpeedLeft, double dSpeedRight);

	/**
	 * Update the robot's pose using Odometry by means of measured encoder tics.
	 */
	void updatePose();

	/**
	 * Set the internal pose estimation to a desired value, e.g.
	 * for specifying a start pose or to correct the odometric estimation
	 * by a pose estimation obtained from another technique (e.g. SLAM).
	 * @param pstrRobotPose Pointer to a pose struct containing the values to adopt.
	 * @return void
	 */
	void setRobotPose(StrPose* pstrRobotPose);

	/**
	 * Set the internal pose estimation to a desired value, e.g.
	 * for specifying a start pose or to correct the odometric estimation
	 * by a pose estimation obtained from another technique (e.g. SLAM).
	 * This function is particularly designed for the use in flat 2D environments
	 * since it only allows to change the robot pose in the 2D plane
	 * (X and Y coordinates as well as the orientation, i.e. rotation around Z-axis)
	 * @param dX X-coordinate of the robot's position to use.
	 * @param dY Y-coordinate of the robot's position to use.
	 * @param dYaw Yaw angle (in rad) specifying the orientation to use.
	 * @return void
	 */
	void setRobotPose(double dX, double dY, double dYaw);

	/**
	 * Reset the robot's pose estimation.
	 * This function reset the pose vector (X-, Y- and Z- coornatines together
	 * with the rotation around these axis) back to the NULL-vector, i.e.
	 * to the starting position.
	 */
	void resetRobotPose();

	/**
	 * Retrieve the number of measured encoder tics from the last pose update.
	 * @param pnRelativeEncoderTicsLeft Pointer to variable to store the number of encoder tics measured at the left wheel.
	 * @param pnRelativeEncoderTicsRight Pointer to variable to store the number of encoder tics measured at the right wheel.
	 * @return void
	 */
	void retrieveRelativeEncoderTics(int &pnRelativeEncoderTicsLeft, int &pnRelativeEncoderTicsRight);

	/**
	 * Retrieve the number of absolute encoder tics (measured since startup)
	 * @param pnRelativeEncoderTicsLeft Pointer to variable to store the number of encoder tics measured at the left wheel.
	 * @param pnRelativeEncoderTicsRight Pointer to variable to store the number of encoder tics measured at the right wheel.
	 * @return void
	 */
	void retrieveAbsoluteEncoderTics(int &pnAbsoluteEncoderTicsLeft, int &pnAbsoluteEncoderTicsRight);

	/**
	 * local struct containing important information about the differential drive robot platform,
	 * e.g. axis length, wheel circumferences or whether and which motor values need to be inverted.
	 */
	StrDifferentialDriveRobotProperties* _pstrDifferentialDriveRobotProperties;

 protected:

	/**
	 * Read the tics measured by wheel encoders,
	 * @param unTicsLeft Number of tics measured by the wheel encoder mounted at the left wheel
	 * @param unTicsRight Number of tics measured by the wheel encoder mounted at the right wheel
	 */
	virtual void getEncoderTics(int *nTicsLeft, int *nTicsRight) = 0;

	double _dEstimatedSpeedLeft;
	double _dEstimatedSpeedRight;

	int _nRelativeEncoderTicsLeft;
	int _nRelativeEncoderTicsRight;

	int _nAbsoluteEncoderTicsLeft;
	int _nAbsoluteEncoderTicsRight;

	fair::CTimer _timerUpdatePose;
	fair::CTimer _timerUpdateMovement;
	fair::CTimer _timerRobotMovementUpdate;

};

#endif /*CROBOTPLATFORMDIFFERENTIALDRIVE_H_*/
