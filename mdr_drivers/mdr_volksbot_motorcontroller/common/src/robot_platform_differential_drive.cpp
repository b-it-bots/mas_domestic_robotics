#include "robot_platform_differential_drive.h"

CRobotPlatformDifferentialDrive::CRobotPlatformDifferentialDrive()
{
	this->_dEstimatedSpeedLeft = 0.0f;
	this->_dEstimatedSpeedRight = 0.0f;

	this->_nRelativeEncoderTicsLeft = 0;
	this->_nRelativeEncoderTicsRight = 0;
	this->_nAbsoluteEncoderTicsLeft = 0;
	this->_nAbsoluteEncoderTicsRight = 0;

	this->_pstrDifferentialDriveRobotProperties = new StrDifferentialDriveRobotProperties();
	this->_timerRobotMovementUpdate.reset();

}

CRobotPlatformDifferentialDrive::~CRobotPlatformDifferentialDrive()
{
	delete _pstrDifferentialDriveRobotProperties;
}

void CRobotPlatformDifferentialDrive::updatePose()
{

	// check time since last call and return if necessary
	long double ldTimeSinceLastUpdate = this->_timerUpdatePose.getTime();
//	if ( ldTimeSinceLastUpdate <  this->_ldPoseUpdateInterval)
//		return;

// reset timer and estimated pose shift
	this->_pstrEstimatedRobotPoseShift->dX = 0.0;
	this->_pstrEstimatedRobotPoseShift->dY = 0.0;
	this->_pstrEstimatedRobotPoseShift->dZ = 0.0;
	this->_pstrEstimatedRobotPoseShift->dRoll = 0.0;
	this->_pstrEstimatedRobotPoseShift->dPitch = 0.0;
	this->_pstrEstimatedRobotPoseShift->dYaw = 0.0;

	// get counted encoder tics (and return if the robot did not move at all)
	int nTicsLeft = 0, nTicsRight = 0;
	this->getEncoderTics(&nTicsLeft, &nTicsRight);
	if ((nTicsLeft == 0) && (nTicsRight == 0))
		return;

	this->_timerUpdatePose.reset();

	// flip motor tics if they are measured in opposite direction
	if (this->_pstrDifferentialDriveRobotProperties->bLeftMotorInverted)
		nTicsLeft = -nTicsLeft;
	if (this->_pstrDifferentialDriveRobotProperties->bRightMotorInverted)
		nTicsRight = -nTicsRight;

	// store encoder tics
	this->_nRelativeEncoderTicsLeft = nTicsLeft;
	this->_nRelativeEncoderTicsRight = nTicsRight;
	this->_nAbsoluteEncoderTicsLeft += nTicsLeft;
	this->_nAbsoluteEncoderTicsRight += nTicsRight;

	// calculate distance travelled by the individual wheels
	double dDistance = 0;
	double dDistanceLeft = nTicsLeft * this->_pstrDifferentialDriveRobotProperties->dWheelCircumferenceLeft
	        / (double) (this->_pstrDifferentialDriveRobotProperties->unTicsPerTurnOfWheelLeft);
	double dDistanceRight = nTicsRight * this->_pstrDifferentialDriveRobotProperties->dWheelCircumferenceRight
	        / (double) (this->_pstrDifferentialDriveRobotProperties->unTicsPerTurnOfWheelRight);

	// check for driving almost straight forward or straight backward
	if (fabs(dDistanceLeft - dDistanceRight) < ROBOT_DIFFERENTIAL_DRIVE_ODOMETRY_EPSILON)
	{

		// (additional) check for not driving at all
		if (fabs(dDistanceLeft) < ROBOT_DIFFERENTIAL_DRIVE_ODOMETRY_EPSILON)
		{
			this->_pstrEstimatedRobotPoseShift->dX = 0.0;
			this->_pstrEstimatedRobotPoseShift->dY = 0.0;
		}
		// straight forward or straight backward(!)
		else
		{
			this->_pstrEstimatedRobotPoseShift->dX = (dDistanceLeft + dDistanceRight) * 0.5;
			this->_pstrEstimatedRobotPoseShift->dY = 0.0;
			dDistance = this->_pstrEstimatedRobotPoseShift->dX;
		}
	}
	// the robot is not moving straight forward or backward(!)
	else
	{
		this->_pstrEstimatedRobotPoseShift->dYaw = ((dDistanceRight - dDistanceLeft) / this->_pstrDifferentialDriveRobotProperties->dAxisLength)
		        * this->_pstrDifferentialDriveRobotProperties->dTurningAdaptation;
		dDistance = (dDistanceRight + dDistanceLeft) * 0.5;
		this->_pstrEstimatedRobotPoseShift->dX = dDistance * cos(this->_pstrEstimatedRobotPoseShift->dYaw);
		this->_pstrEstimatedRobotPoseShift->dY = dDistance * sin(this->_pstrEstimatedRobotPoseShift->dYaw);
	}

	if ((this->_pstrEstimatedRobotPoseShift->dX + this->_pstrEstimatedRobotPoseShift->dY) > 1000)
	{
		printf("ERROR: CRobotPlatformDifferentialDrive measured weird encoder tics!\n");
		return;
	}

	// update pose
	double dCosYaw = cos(this->_pstrEstimatedRobotPose->dYaw);
	double dSinYaw = sin(this->_pstrEstimatedRobotPose->dYaw);
	this->_pstrEstimatedRobotPose->dX = this->_pstrEstimatedRobotPoseShift->dX * dCosYaw - this->_pstrEstimatedRobotPoseShift->dY * dSinYaw
	        + this->_pstrEstimatedRobotPose->dX;
	this->_pstrEstimatedRobotPose->dY = this->_pstrEstimatedRobotPoseShift->dX * dSinYaw + this->_pstrEstimatedRobotPoseShift->dY * dCosYaw
	        + this->_pstrEstimatedRobotPose->dY;
	double dCorrectedAngle = this->_pstrEstimatedRobotPose->dYaw + this->_pstrEstimatedRobotPoseShift->dYaw;
	this->_pstrEstimatedRobotPose->dYaw = dCorrectedAngle;

	// correct orientation / limit orientation to interval [-pi pi]
	if (this->_pstrEstimatedRobotPose->dYaw > M_PI)
		this->_pstrEstimatedRobotPose->dYaw -= 2.0 * M_PI;
	else if (_pstrEstimatedRobotPose->dYaw < -M_PI)
		this->_pstrEstimatedRobotPose->dYaw += 2.0 * M_PI;

	// calculate translational and angular velocities
	this->_dEstimatedSpeedLeft = dDistanceLeft / ldTimeSinceLastUpdate;
	this->_dEstimatedSpeedRight = dDistanceRight / ldTimeSinceLastUpdate;
	this->_dEstimatedSpeedAngular = this->_pstrEstimatedRobotPoseShift->dYaw / ldTimeSinceLastUpdate;
	this->_dEstimatedSpeedTranslational = dDistance / ldTimeSinceLastUpdate;
}

void CRobotPlatformDifferentialDrive::move(double dTranslationalVelocity, double dAngularVelocity)
{
	if (this->_timerRobotMovementUpdate.getTime() < this->getMoveUpdateInterval())
		return;

	// check time since last call and return if necessary
//	long double ldTimeSinceLastUpdate = _timerUpdateMovement.getTime();
//	if ( ldTimeSinceLastUpdate <  _ldMoveUpdateInterval)
//		return;

// reset timer and estimated pose shift
	_timerUpdateMovement.reset();

	double dTranslationalVelocityToSet = dTranslationalVelocity;
	double dAngularVelocityToSet = dAngularVelocity;

	// robot is NOT allowed to move backward
//	if ( dTranslationalVelocityToSet < 0.0001 )
//		dTranslationalVelocityToSet = 0;

	double dSpeedLeft = 0.0f;
	double dSpeedRight = 0.0f;

	// calculate speed of left and right wheel
//	double dSpeedDifference = ( dAngularVelocityToSet / 4.0 * (_pstrDifferentialDriveRobotProperties->dAxisLength * 0.001) );  // Axis length from mm to m 
//	dSpeedLeft  = (double)(dTranslationalVelocity - fabs(dSpeedDifference) - dSpeedDifference);
//	dSpeedRight = (double)(dTranslationalVelocity - fabs(dSpeedDifference) + dSpeedDifference);
//	std::cout << dAngularVelocityToSet << "   :   " << dSpeedDifference << std::endl;

// alternative mapping from velocities to left and right wheel speeds
	double dSpeedDifference = dAngularVelocityToSet * _pstrDifferentialDriveRobotProperties->dAxisLength * 0.0005;  // omega * b/2
	double dMinAngularVelocity = 0.0001;
	double dMinTranslationalVelocity = 0.0001;

	// TODO: do we need to limit the speed difference? speeds are limited anyway!!!
	dSpeedDifference = std::min(dSpeedDifference, 2.0 * this->_pstrRobotProperties->dMaxTranslationalVelocity);
	dSpeedDifference = std::max(dSpeedDifference, -2.0 * this->_pstrRobotProperties->dMaxTranslationalVelocity);

	if (fabs(dTranslationalVelocityToSet) < dMinTranslationalVelocity)
	{
		if (fabs(dAngularVelocityToSet) < dMinAngularVelocity)  // robot is requested to (almost) stand still
		{
			dSpeedLeft = 0.0f;
			dSpeedRight = 0.0f;
		}
		else  // robot is requested to 'turn on the spot'
		{
			dSpeedLeft = -dSpeedDifference;
			dSpeedRight = +dSpeedDifference;
		}
	}
	else
	{
		if (fabs(dAngularVelocityToSet) < dMinAngularVelocity)  // robot is requested to drive on a straight ahead
		{
			dSpeedLeft = dTranslationalVelocityToSet;
			dSpeedRight = dTranslationalVelocityToSet;
		}
		else  // 'normal' movement
		{
			dSpeedLeft = dTranslationalVelocityToSet - dSpeedDifference;
			dSpeedRight = dTranslationalVelocityToSet + dSpeedDifference;

//			// BEGIN SHIFTING! TODO: reasonable???? shift speeds to avoid that one wheel is moving backward!
//			if ( dTranslationalVelocityToSet >= 0 )  // forward movement
//			{
//				if ( dSpeedLeft < 0 ) // avoid that left wheel moves backward
//				{
////					dSpeedRight += fabs(dSpeedLeft);
//					dSpeedLeft = 0;
//				}
//				if ( dSpeedRight < 0 ) // avoid that right wheel moves backward
//				{
////					dSpeedLeft += fabs(dSpeedRight);
//					dSpeedRight = 0;
//				}
//			}
//			else // backward movement
//			{
//				if ( dSpeedLeft > 0 ) // avoid that left wheel moves forward
//				{
////					dSpeedRight -= fabs(dSpeedLeft);
//					dSpeedLeft = 0;
//				}
//				if ( dSpeedRight > 0 ) // avoid that right wheel moves forward
//				{
////					dSpeedLeft -= fabs(dSpeedRight);
//					dSpeedRight = 0;
//				}				
//			}
//			// END SHIFTING!

		}
	}

	// scale calculated speeds if one of them exceeds the maximum velocity of the robot
	double dMaxSpeed = MAX(fabs(dSpeedLeft), fabs(dSpeedRight));
	if (dMaxSpeed > _pstrRobotProperties->dMaxTranslationalVelocity)
	{
		double dScaleFactor = fabs(_pstrRobotProperties->dMaxTranslationalVelocity / dMaxSpeed);
		dSpeedLeft *= dScaleFactor;
		dSpeedRight *= dScaleFactor;
	}

	// flip motor speeds if they are measured in opposite direction
	if (this->_pstrDifferentialDriveRobotProperties->bLeftMotorInverted)
		dSpeedLeft = -dSpeedLeft;
	if (this->_pstrDifferentialDriveRobotProperties->bRightMotorInverted)
		dSpeedRight = -dSpeedRight;

	// set movement speed
	this->setSpeed(dSpeedLeft, dSpeedRight);

	this->updatePose();

	this->_timerRobotMovementUpdate.reset();

//	std::cout << "CRobotPlatformDifferentialDrive::move(): " << dTranslationalVelocity << " " << dAngularVelocityToSet << "    " <<  dSpeedLeft << " " << dSpeedRight << std::endl << std::endl;
}

void CRobotPlatformDifferentialDrive::setSpeedFactors(double dSpeedLeft, double dSpeedRight)
{
	// flip motor speeds if they are measured in opposite direction
	if (this->_pstrDifferentialDriveRobotProperties->bLeftMotorInverted)
		dSpeedLeft = -dSpeedLeft;
	if (this->_pstrDifferentialDriveRobotProperties->bRightMotorInverted)
		dSpeedRight = -dSpeedRight;

	this->setSpeed(dSpeedLeft * this->_pstrRobotProperties->dMaxTranslationalVelocity, dSpeedRight * this->_pstrRobotProperties->dMaxTranslationalVelocity);
}

void CRobotPlatformDifferentialDrive::setRobotPose(StrPose* pstrRobotPose)
{
	this->_pstrEstimatedRobotPose->dX = pstrRobotPose->dX;
	this->_pstrEstimatedRobotPose->dY = pstrRobotPose->dY;
	this->_pstrEstimatedRobotPose->dZ = pstrRobotPose->dZ;
	this->_pstrEstimatedRobotPose->dRoll = pstrRobotPose->dRoll;
	this->_pstrEstimatedRobotPose->dPitch = pstrRobotPose->dPitch;
	this->_pstrEstimatedRobotPose->dYaw = pstrRobotPose->dYaw;
}

void CRobotPlatformDifferentialDrive::setRobotPose(double dX, double dY, double dYaw)
{
	this->_pstrEstimatedRobotPose->dX = dX;
	this->_pstrEstimatedRobotPose->dY = dY;
	this->_pstrEstimatedRobotPose->dZ = 0;
	this->_pstrEstimatedRobotPose->dRoll = 0;
	this->_pstrEstimatedRobotPose->dPitch = 0;
	this->_pstrEstimatedRobotPose->dYaw = dYaw;
}

void CRobotPlatformDifferentialDrive::resetRobotPose()
{
	this->_pstrEstimatedRobotPose->dX = 0;
	this->_pstrEstimatedRobotPose->dY = 0;
	this->_pstrEstimatedRobotPose->dZ = 0;
	this->_pstrEstimatedRobotPose->dRoll = 0;
	this->_pstrEstimatedRobotPose->dPitch = 0;
	this->_pstrEstimatedRobotPose->dYaw = 0;
}

void CRobotPlatformDifferentialDrive::retrieveRelativeEncoderTics(int &pnRelativeEncoderTicsLeft, int &pnRelativeEncoderTicsRight)
{
	pnRelativeEncoderTicsLeft = this->_nRelativeEncoderTicsLeft;
	pnRelativeEncoderTicsRight = this->_nRelativeEncoderTicsRight;
}

void CRobotPlatformDifferentialDrive::retrieveAbsoluteEncoderTics(int &pnAbsoluteEncoderTicsLeft, int &pnAbsoluteEncoderTicsRight)
{
	pnAbsoluteEncoderTicsLeft = this->_nAbsoluteEncoderTicsLeft;
	pnAbsoluteEncoderTicsRight = this->_nAbsoluteEncoderTicsRight;
}

