/*
 * CRobotFactory.h
 *
 *  Created on: Nov 14, 2011
 *      Author: Frederik
 */

#ifndef CROBOTFACTORY_H_
#define CROBOTFACTORY_H_

#include <stdio.h>
#include <string.h>
#include "robot_platform_volksbot_rt.h"

using namespace std;

class CRobotFactory
{
 public:
	CRobotFactory();
	~CRobotFactory();

	bool connect(const string device_name);
	CRobotPlatformDifferentialDrive* getRobotPlatformDifferentialDrive()
	{
		return _robot_platform_differential_drive;
	}
	;
	void getBatteryStatus(int& battery_percentage, bool& is_charging);

 private:
	void setParameters();

	CRobotPlatformDifferentialDrive* _robot_platform_differential_drive;
	CRobotPlatformVolksbotRT* _robot_platform_volksbot_rt;
	fair::CTimer _timerBatteryMonitor;
	int _battery_percentage;
	bool _is_charging;
};

#endif /* CROBOTFACTORY_H_ */
