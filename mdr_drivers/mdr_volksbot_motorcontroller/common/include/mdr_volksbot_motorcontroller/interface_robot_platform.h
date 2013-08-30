#ifndef IROBOTPLATFORM_H_
#define IROBOTPLATFORM_H_

#include <iostream>
#include <fstream>

#include "core_data_types.h"
#include "core_functions.h"
#include "core_defines.h"

#include "log.h"
#include "timer.h"

/**
 * @struct StrRobotProperties
 * @brief Generic properties of a pobot platform
 * @author Dirk Holz, Fraunhofer IAIS, August 2007
 */
struct StrRobotProperties
{
	StrRobotProperties()
	{
		dBoundingBoxDistanceFront = 0.0f;
		dBoundingBoxDistanceTail = 0.0f;
		dBoundingBoxDistanceLeft = 0.0f;
		dBoundingBoxDistanceRight = 0.0f;
		dBoundingBoxHeight = 0.0f;

		dMinTranslationalVelocity = 0.0f;
		dMaxTranslationalVelocity = 0.0f;
		dMaxAngularVelocity = 0.0f;
	}
	;

	double dBoundingBoxDistanceFront;
	double dBoundingBoxDistanceTail;
	double dBoundingBoxDistanceLeft;
	double dBoundingBoxDistanceRight;
	double dBoundingBoxHeight;

	double dMinTranslationalVelocity;
	double dMaxTranslationalVelocity;
	double dMaxAngularVelocity;

};

/**
 * @class IRobotPlatform
 * @brief Abstract Implementation of a robot platform allowing to move
 * 	robots and access their position and orientation in space
 * @author Dirk Holz, Fraunhofer IAIS, August 2007
 */
class IRobotPlatform
{
 public:
	IRobotPlatform()
	{
		_pstrEstimatedRobotPose = new StrPose();
		_pstrEstimatedRobotPoseShift = new StrPose();
		_pstrRobotProperties = new StrRobotProperties();
		_dTranslationalVelocityToSet = 0;
		_dAngularVelocityToSet = 0;
	}
	;
	virtual ~IRobotPlatform()
	{
		delete _pstrEstimatedRobotPose;
		delete _pstrEstimatedRobotPoseShift;
		delete _pstrRobotProperties;
	}
	;

	void move()
	{
		this->move(this->_dTranslationalVelocityToSet, this->_dAngularVelocityToSet);
	}
	;
	void stop()
	{
		this->move(0.0f, 0.0f);
	}
	;

	virtual void move(double dTranslationalVelocity, double dAngularVelocity) = 0;
	virtual void updatePose() = 0;

	void moveRelative(double dTranslationalVelocity, double dAngularVelocity)
	{
		this->move(dTranslationalVelocity * this->_pstrRobotProperties->dMaxTranslationalVelocity,
		           dAngularVelocity * this->_pstrRobotProperties->dMaxAngularVelocity);
	}
	;

	virtual void setRobotPose(StrPose* pstrRobotPose) = 0;
	virtual void setRobotPose(double dX, double dY, double dYaw) = 0;
	virtual void resetRobotPose() = 0;

	StrPose* getPose()
	{
		return _pstrEstimatedRobotPose;
	}
	;
	StrPose* getPoseSihft()
	{
		return _pstrEstimatedRobotPoseShift;
	}
	;

	void setPoseUpdateInterval(long double ldPoseUpdateInterval)
	{
		_ldPoseUpdateInterval = ldPoseUpdateInterval;
	}
	;
	long double getPoseUpdateInterval()
	{
		return _ldPoseUpdateInterval;
	}
	;

	void setMoveUpdateInterval(long double ldMoveUpdateInterval)
	{
		_ldMoveUpdateInterval = ldMoveUpdateInterval;
	}
	;
	long double getMoveUpdateInterval()
	{
		return _ldMoveUpdateInterval;
	}
	;

	double getEstimatedAngularSpeed()
	{
		return _dEstimatedSpeedAngular;
	}
	;
	double getEstimatedTranslationalSpeed()
	{
		return _dEstimatedSpeedTranslational;
	}
	;

	StrRobotProperties* _pstrRobotProperties;

	double _dTranslationalVelocityToSet;
	double _dAngularVelocityToSet;

 protected:
	StrPose* _pstrEstimatedRobotPose;
	StrPose* _pstrEstimatedRobotPoseShift;

	long double _ldPoseUpdateInterval;
	long double _ldMoveUpdateInterval;

	double _dEstimatedSpeedAngular;
	double _dEstimatedSpeedTranslational;

};

#endif /*IROBOTPLATFORM_H_*/
