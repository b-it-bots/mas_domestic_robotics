#include "robot_platform_volksbot_rt.h"

CRobotPlatformVolksbotRT::CRobotPlatformVolksbotRT(unsigned int unMotorNumberLeft, unsigned int unMotorNumberRight)
{
	_unMotorNumberLeft = unMotorNumberLeft;
	_unMotorNumberRight = unMotorNumberRight;

//	fileDebugOutput.open("volksbot_controller_debug.dat", std::ios::trunc);
}

CRobotPlatformVolksbotRT::~CRobotPlatformVolksbotRT()
{
	delete _pVmcApi;
//	fileDebugOutput.close();
}

int CRobotPlatformVolksbotRT::connect(char* szDeviceName)
{

	int nRetVal = FAIR_EXIT_SUCCESS;
	_pVmcApi = new VMC::CVmcApi();
	long double oldTimeStamp = 0.0f;

	_pVmcApi->selectHardwareAdapter(VMC::eAdapterRS232);
	if (!_pVmcApi->selectDevice(szDeviceName))
		return FAIR_EXIT_FAILURE;

	_pVmcApi->configRequestMessage(0, false); 	// deactivate RPM in request list
	_pVmcApi->configRequestMessage(3, true); 	// activate AbsoluteRotations in request list (abolsute rotations = absolute tics!)
	_pVmcApi->configRequestMessage(7, true);

	//  	_pVmcApi->useVMC().ClearAllAbsolutRotations.Set(1, 1, 1);
	_pVmcApi->useVMC().Motor[_unMotorNumberLeft].AbsolutRotations.Update();
	_pVmcApi->useVMC().Motor[_unMotorNumberRight].AbsolutRotations.Update();
	_pVmcApi->useVMC().MotorRPMs.Set(0.0f, 0.0f, 0.0f);
	FAIR_SLEEP(100);  // default 50, wait 50ms
	_dAbsoluteTicsLeft = _pVmcApi->useVMC().Motor[_unMotorNumberLeft].AbsolutRotations.getValue();
	_dAbsoluteTicsRight = _pVmcApi->useVMC().Motor[_unMotorNumberRight].AbsolutRotations.getValue();

	oldTimeStamp = _pVmcApi->useVMC().Motor[_unMotorNumberLeft].AbsolutRotations.getTimestamp();
	FAIR_SLEEP(200);  // default 50, wait 50ms

	_pVmcApi->useVMC().Motor[_unMotorNumberLeft].AbsolutRotations.Update();
	_pVmcApi->useVMC().Motor[_unMotorNumberRight].AbsolutRotations.Update();
	_pVmcApi->useVMC().MotorRPMs.Set(0.0f, 0.0f, 0.0f);
	FAIR_SLEEP(100);  // default 50, wait 50ms
	_dAbsoluteTicsLeft = _pVmcApi->useVMC().Motor[_unMotorNumberLeft].AbsolutRotations.getValue();
	_dAbsoluteTicsRight = _pVmcApi->useVMC().Motor[_unMotorNumberRight].AbsolutRotations.getValue();

	if (oldTimeStamp == _pVmcApi->useVMC().Motor[_unMotorNumberLeft].AbsolutRotations.getTimestamp())
		return FAIR_EXIT_FAILURE;

	//_pVmcApi->printRequestMessage();

	return nRetVal;
}

void CRobotPlatformVolksbotRT::setSpeed(double dSpeedLeft, double dSpeedRight)
{
	// input: velocities in m/s

	// m/s = mm/ms

//	double dUnknownConstant = 74.0;
//	double dFactorToRoundsPerMinute = 6000.0;
//	double dRPMLeft = dSpeedLeft * dUnknownConstant * dFactorToRoundsPerMinute / this->_pstrDifferentialDriveRobotProperties->dWheelCircumferenceLeft;
//	double dRPMRight = dSpeedRight * dUnknownConstant * dFactorToRoundsPerMinute / this->_pstrDifferentialDriveRobotProperties->dWheelCircumferenceRight;

	// speed [mm / ms] / wheel circumference [mm]
	double dGearRatio = 73.5;  // READ FROM CONFIG FILE?!?!?!?!?!?!?
//	double dWheelCircumference = this->_pstrDifferentialDriveRobotProperties->dWheelCircumferenceLeft / 1000.0; // m
//	double dFactorToRoundsPerMinute = (1 / dWheelCircumference) * dGearRatio * 60.0;
	double dWheelCircumference = this->_pstrDifferentialDriveRobotProperties->dWheelCircumferenceLeft;
	double dFactorToRoundsPerMinute = (1000 / dWheelCircumference) * dGearRatio * 60.0;
	double dRPMLeft = dSpeedLeft * dFactorToRoundsPerMinute;
	double dRPMRight = dSpeedRight * dFactorToRoundsPerMinute;

	double dSetSpeeds[3] =
	{ 0.0f, 0.0f, 0.0f };
	dSetSpeeds[_unMotorNumberLeft] = dRPMLeft;
	dSetSpeeds[_unMotorNumberRight] = dRPMRight;

	_pVmcApi->useVMC().MotorRPMs.Set(dSetSpeeds[0], dSetSpeeds[1], dSetSpeeds[2]);

//	// reading motor currents, NOT NEEDED FOR ANYTHING
//	_pVmcApi->useVMC().Motor[_unMotorNumberLeft].ActualCurrent.Update();
//    _pVmcApi->useVMC().Motor[_unMotorNumberRight].ActualCurrent.Update();
//	double dCurrentLeft = _pVmcApi->useVMC().Motor[_unMotorNumberLeft].ActualCurrent.getValue();
//	double dCurrentRight = _pVmcApi->useVMC().Motor[_unMotorNumberRight].ActualCurrent.getValue();
//	printf("currents: %0.4lf %0.4lf\n", dCurrentLeft, dCurrentRight);

//	double dActualRPMLeft =  _pVmcApi->useVMC().Motor[_unMotorNumberLeft].ActualRPM.getValue();
//	double dActualRPMRight = _pVmcApi->useVMC().Motor[_unMotorNumberRight].ActualRPM.getValue();

//	if ( fileDebugOutput.is_open() )
//	{
//		std::cout << "CRobotPlatformVolksbotRT: set:" << dSpeedLeft << ' ' << dSpeedRight
//			<< " mm/ms  estimated: " << this->_dEstimatedSpeedLeft << ' ' << this->_dEstimatedSpeedRight
//			<< " rpm: " << dRPMLeft << ' ' << dRPMRight << std::endl;
	//<< "  act: " << dActualRPMLeft << ' ' << dActualRPMRight << std::endl;
//		fileDebugOutput << timerDebugOutput.getTime() << ' '
//			<< dSpeedLeft << ' ' << dSpeedRight << ' '
//			<< dRPMLeft << ' ' << dRPMRight << ' '
//			<< dActualRPMLeft << ' ' << dActualRPMRight << std::endl;
//	}

}

void CRobotPlatformVolksbotRT::getEncoderTics(int *nTicsLeft, int *nTicsRight)
{
	static long double oldTimeStamp = 0.0f;
	double dCurrentAbsoluteTicsLeft = _pVmcApi->useVMC().Motor[_unMotorNumberLeft].AbsolutRotations.getValue();
	double dCurrentAbsoluteTicsRight = _pVmcApi->useVMC().Motor[_unMotorNumberRight].AbsolutRotations.getValue();

	if ((_pVmcApi->useVMC().Motor[_unMotorNumberLeft].AbsolutRotations.getTimestamp() - oldTimeStamp) > 500)
	{
		_dAbsoluteTicsLeft = dCurrentAbsoluteTicsLeft;
		_dAbsoluteTicsRight = dCurrentAbsoluteTicsRight;
	}

	oldTimeStamp = _pVmcApi->useVMC().Motor[_unMotorNumberLeft].AbsolutRotations.getTimestamp();

	(*nTicsLeft) = (int) (dCurrentAbsoluteTicsLeft - _dAbsoluteTicsLeft);
	(*nTicsRight) = (int) (dCurrentAbsoluteTicsRight - _dAbsoluteTicsRight);

	int nTicsMeasuredLeft = (int) (dCurrentAbsoluteTicsLeft - _dAbsoluteTicsLeft);
	int nTicsMeasuredRight = (int) (dCurrentAbsoluteTicsRight - _dAbsoluteTicsRight);
	if (fabs(nTicsMeasuredLeft) > 2000000000)
		nTicsMeasuredLeft = (int) (dCurrentAbsoluteTicsLeft + _dAbsoluteTicsLeft);
	if (fabs(nTicsMeasuredRight) > 2000000000)
		nTicsMeasuredRight = (int) (dCurrentAbsoluteTicsRight + _dAbsoluteTicsRight);

	this->_dAbsoluteTicsLeft = dCurrentAbsoluteTicsLeft;
	this->_dAbsoluteTicsRight = dCurrentAbsoluteTicsRight;

	(*nTicsLeft) = nTicsMeasuredLeft;
	(*nTicsRight) = nTicsMeasuredRight;

	/*** DEBUG OUTPUT ***/
	//_pVmcApi->printAllErrors();
	//printf("CRobotPlatformVolksbotRT::getEncoderTics: %d %d (%0.2lf %0.2lf --- %0.2lf %0.2lf).\n", (*nTicsLeft), (*nTicsRight), this->_dAbsoluteTicsLeft, this->_dAbsoluteTicsRight, dCurrentAbsoluteTicsLeft, dCurrentAbsoluteTicsRight);
	return;
}

void CRobotPlatformVolksbotRT::getBatteryStatus(int& piBatteryPercentage, bool& bCharging)
{
	bool charging = false;
	int BatteryPercent = 0;

	double BatteryVoltage = 0;
	double charge_BatteryVoltage = 27000;
	double max_BatteryVoltage = 25700;
	double normal_BatteryVoltage = 24300;
	double low_BatteryVoltage = 23000;
	double empty_BatteryVoltage = 20000;

	//_pVmcApi->useVMC().BatteryVoltage.Update();
	// std::cout << "Timestamp: " << Controller.useVMC().BatteryVoltage.getTimestamp()
	//  <<" Value: "<< Controller.useVMC().BatteryVoltage.getValue() << std::endl;
	BatteryVoltage = _pVmcApi->useVMC().BatteryVoltage.getValue();

	if (BatteryVoltage >= charge_BatteryVoltage)
	{
		charging = true;
		BatteryPercent = 100;
		//std::cout << "Charging" << " volt: " << BatteryVoltage << std::endl;
	}
	else if (BatteryVoltage > normal_BatteryVoltage && BatteryVoltage <= max_BatteryVoltage)
	{
		BatteryPercent = (int) (((BatteryVoltage - normal_BatteryVoltage) / (max_BatteryVoltage - normal_BatteryVoltage)) * 20) + 80;
		//std::cout << "not charging battery Percent: " << BatteryPercent << " volt: " << BatteryVoltage << std::endl;
		charging = false;
	}
	else if (BatteryVoltage > low_BatteryVoltage && BatteryVoltage <= normal_BatteryVoltage)
	{
		BatteryPercent = (int) (((BatteryVoltage - low_BatteryVoltage) / (normal_BatteryVoltage - low_BatteryVoltage)) * 70) + 10;
		//std::cout << "not charging battery Percent: " << BatteryPercent << " volt: " << BatteryVoltage << std::endl;
		charging = false;
	}
	else if (BatteryVoltage > empty_BatteryVoltage && BatteryVoltage <= low_BatteryVoltage)
	{
		BatteryPercent = (int) (((BatteryVoltage - low_BatteryVoltage) / (normal_BatteryVoltage - low_BatteryVoltage)) * 10);
		//std::cout << "not charging battery Percent: " << BatteryPercent << " volt: " << BatteryVoltage << std::endl;
		charging = false;
	}

	piBatteryPercentage = BatteryPercent;
	bCharging = charging;
}

