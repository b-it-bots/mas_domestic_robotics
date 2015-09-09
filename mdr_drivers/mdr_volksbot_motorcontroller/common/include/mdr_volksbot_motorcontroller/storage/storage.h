//////////////////////////////////////////////////////////////////////////////
///  @file CStorage.h
///  @class VMC::CStorage
///  @brief This class contain all parameters and runtime variables which belong to the whole contoller.
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////

#ifndef _CStorage_H_
#define _CStorage_H_

#include <vector>

#include <mdr_volksbot_motorcontroller/storage/motor.h>
#include <mdr_volksbot_motorcontroller/support/data.h>
#include <mdr_volksbot_motorcontroller/support/multisend.h>
#include <mdr_volksbot_motorcontroller/support/send_two.h>
#include <mdr_volksbot_motorcontroller/support/enums.h>
#include <mdr_volksbot_motorcontroller/support/channel.h>
#include <mdr_volksbot_motorcontroller/support/message.h>
#include <mdr_volksbot_motorcontroller/support/commands.h>

namespace VMC
{

	class CStorage
	{
 	public:
		CStorage()
		{
			m_bInitialized = false;
		}

		~CStorage()
		{
		}
		;

		bool Init(CTranslationLayer* Trans, BYTE* pNextRequestCommand);

		std::vector<VMC::CMotor> Motor;

		CData BumperActiv;
		CData BatteryVoltage;
		CData VMC_Time;
//	CData VelocityInputType;	
		CData VMCTimeout;
		CData VMCVersion;
		CData LoadParameters;
		CData SaveParameters;

		CMultisend MotorRPMs;
		CMultisend MotorPWMs;
		CSendTwo RobotVelocity;

		CData IOPortConfiguration;
//	CData ClearAllAbsolutRotations;  gone to multisend
		CMultisend ClearAllAbsolutRotations;
		CMultisend BumperOn;
		CData DigitalIN;
		CData DigitalOUT;  // gone to multisend
//	CMultisend DigitalOUT;
		CData AnalogInput1;
		CData AnalogInput2;

 	private:
		bool m_bInitialized;
		CTranslationLayer* m_pTrans;
	};

}  // namespace VMC
#endif //_CStorage_H_
