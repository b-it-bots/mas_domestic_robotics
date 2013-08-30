#ifndef CROBOTPLATFORMVOLKSBOTRT_H_
#define CROBOTPLATFORMVOLKSBOTRT_H_

#include <iostream>
#include <fstream>

#include "timer.h"
#include "common.h"

#include "robot_platform_differential_drive.h"
#include "layer/vmc_api.h"

class CRobotPlatformVolksbotRT : public CRobotPlatformDifferentialDrive
{
 public:
	CRobotPlatformVolksbotRT(unsigned int unMotorNumberLeft = 0, unsigned int unMotorNumberRight = 1);
	~CRobotPlatformVolksbotRT();

	int connect(char* szDeviceName = "/dev/ttyS0");
	void setSpeed(double dSpeedLeft, double dSpeedRight);
	void getBatteryStatus(int& piBatteryPercentage, bool& pbCharging);
 private:

	void getEncoderTics(int *nTicsLeft, int *nTicsRight);

	VMC::CVmcApi* _pVmcApi;

	double _dAbsoluteTicsLeft;
	double _dAbsoluteTicsRight;

	unsigned int _unMotorNumberLeft;
	unsigned int _unMotorNumberRight;

	std::ofstream fileDebugOutput;
	fair::CTimer timerDebugOutput;

};

#endif /*CROBOTPLATFORMVOLKSBOTRT_H_*/

