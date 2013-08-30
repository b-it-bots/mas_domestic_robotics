/*
 * CRobotFactory.cpp
 *
 *  Created on: Nov 14, 2011
 *      Author: Frederik Hegger
 */

#include "robot_factory.h"

CRobotFactory::CRobotFactory()
{
	cout << "CRobotFactory: Creating an instance of a differential drive robot" << endl;
	this->_robot_platform_differential_drive = NULL;
	this->_robot_platform_volksbot_rt = NULL;
	this->_timerBatteryMonitor.reset();
	this->_battery_percentage = 0;
	this->_is_charging = false;
}

CRobotFactory::~CRobotFactory()
{
	cout << "CRobotFactory: destroy instance of differential drive robot named" << endl;

	if (this->_robot_platform_volksbot_rt != NULL)
		delete this->_robot_platform_volksbot_rt;
	if (this->_robot_platform_differential_drive != NULL)
		delete this->_robot_platform_differential_drive;
}

bool CRobotFactory::connect(const string device_name)
{
	this->_robot_platform_volksbot_rt = new CRobotPlatformVolksbotRT(1, 0);

	if (this->_robot_platform_volksbot_rt->connect(const_cast<char*>(device_name.c_str())) == FAIR_EXIT_SUCCESS)
	{
		cout << "CRobotFactory: Creating an instance of CRobotPlatformVolksbotRT" << endl;
		this->_robot_platform_differential_drive = (CRobotPlatformDifferentialDrive*) this->_robot_platform_volksbot_rt;
		this->setParameters();

		fair::CTimer timerRobotMovementUpdate;
		timerRobotMovementUpdate.reset();
		while (timerRobotMovementUpdate.getTime() < 100)
		{
			this->_robot_platform_differential_drive->move(0, 0);
			this->_robot_platform_differential_drive->updatePose();
		}

		return true;
	}
	else
	{
		delete this->_robot_platform_volksbot_rt;
		cout << "CRobotFactory: Could not connect to CRobotPlatformVolksbotRT on " << device_name << endl;
		return false;
	}
}

void CRobotFactory::getBatteryStatus(int& battery_percentage, bool& is_charging)
{
	if (_timerBatteryMonitor.getTime() > 5000)
	{
		this->_robot_platform_volksbot_rt->getBatteryStatus(this->_battery_percentage, this->_is_charging);
		_timerBatteryMonitor.reset();
	}

	battery_percentage = this->_battery_percentage;
	is_charging = this->_is_charging;
}

void CRobotFactory::setParameters()
{
	if (this->_robot_platform_differential_drive != NULL)
	{
		this->_robot_platform_differential_drive->setPoseUpdateInterval(20);
		this->_robot_platform_differential_drive->setMoveUpdateInterval(20);
		this->_robot_platform_differential_drive->_pstrDifferentialDriveRobotProperties->dAxisLength = 463;
		this->_robot_platform_differential_drive->_pstrDifferentialDriveRobotProperties->dWheelCircumferenceLeft = 534.1;
		this->_robot_platform_differential_drive->_pstrDifferentialDriveRobotProperties->dWheelCircumferenceRight = 534.1;
		this->_robot_platform_differential_drive->_pstrDifferentialDriveRobotProperties->unTicsPerTurnOfWheelLeft = 37000;
		this->_robot_platform_differential_drive->_pstrDifferentialDriveRobotProperties->unTicsPerTurnOfWheelRight = 37000;
		this->_robot_platform_differential_drive->_pstrDifferentialDriveRobotProperties->dTurningAdaptation = 1.0;
		this->_robot_platform_differential_drive->_pstrRobotProperties->dMinTranslationalVelocity = 0;
		this->_robot_platform_differential_drive->_pstrRobotProperties->dMaxTranslationalVelocity = 0.4;
		this->_robot_platform_differential_drive->_pstrRobotProperties->dMaxAngularVelocity = 1.5;
		this->_robot_platform_differential_drive->_pstrRobotProperties->dBoundingBoxDistanceFront = 150.0;
		this->_robot_platform_differential_drive->_pstrRobotProperties->dBoundingBoxDistanceTail = 460.0;
		this->_robot_platform_differential_drive->_pstrRobotProperties->dBoundingBoxDistanceLeft = 270.0;
		this->_robot_platform_differential_drive->_pstrRobotProperties->dBoundingBoxDistanceRight = 270.0;
		this->_robot_platform_differential_drive->_pstrRobotProperties->dBoundingBoxHeight = 1200.0;
		this->_robot_platform_differential_drive->_pstrDifferentialDriveRobotProperties->bLeftMotorInverted = true;
		this->_robot_platform_differential_drive->_pstrDifferentialDriveRobotProperties->bRightMotorInverted = false;
	}
}

