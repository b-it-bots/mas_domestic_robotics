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

#include "motor.h"
#include "support/data.h"
#include "support/multisend.h"
#include "support/send_two.h"
#include "support/enums.h"
#include "support/channel.h"
#include "support/message.h"
#include "support/commands.h"

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
