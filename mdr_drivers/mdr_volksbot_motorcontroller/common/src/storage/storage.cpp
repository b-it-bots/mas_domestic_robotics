//////////////////////////////////////////////////////////////////////////////
///  @file CStorage.cpp
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////

#include "storage/storage.h"

namespace VMC
{

	bool CStorage::Init(CTranslationLayer* Trans, BYTE* pNextRequestCommand)
	{

		if (m_bInitialized)
			return false;

		m_pTrans = Trans;
		CChannel Channel0(0);
		CMotor* MotorPointer;

		BumperActiv.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                 _CMDGRP_MOTOR_CTRL_, _SET_MODE_WITH_BUMPER_,								//Command Group / Command
		                 eDataTypeSignedInteger, Channel0, "BumperActiv",	//Data Type / Motor Channel / Name
		                 0.0, 1.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		BatteryVoltage.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                    _CMDGRP_MOTOR_STATUSIN_, _SREG_BATTERY_,								//Command Group / Command
		                    eDataTypeSignedInteger, Channel0, "BatteryVoltage",  //Data Type / Motor Channel / Name
		                    0.0, 40000.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		VMC_Time.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		              _CMDGRP_MOTOR_STATUSIN_, _SREG_TIME_,								//Command Group / Command
		              eDataTypeSignedLong, Channel0, "VMC-Time",	//Data Type / Motor Channel / Name
		              0.0, 2147483647.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		VMCTimeout.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                _CMDGRP_MOTOR_CONFIN_, _MCMD_TIMEOUT_,								//Command Group / Command
		                eDataTypeSignedInteger, Channel0, "VMCTimeout",  //Data Type / Motor Channel / Name
		                0.0, _TIMEOUT_, 0.0, eUnitPrefixMILLI, ePhysicalUnitSECOND);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		VMCVersion.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                _CMDGRP_BASE_, 0x10,			 					//Command Group / Command
		                eDataTypeString, Channel0, "VMCVersion",			//Data Type / Motor Channel / Name
		                0.0, 0.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		                /*
		                 VelocityInputType.Init(
		                 m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                 _CMDGRP_MOTOR_CONFIN_, _MCMD_USE_PWM_,			 					//Command Group / Command
		                 SignedInteger, Channel0, "VelocityInputType",			//Data Type / Motor Channel / Name
		                 0.0, 1.0, 0.0, one, none);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		                 */
		//12345
		LoadParameters.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                    _CMDGRP_MOTOR_CONFIN_, _MCMD_LOAD_CONFIG_,			 					//Command Group / Command
		                    eDataTypeSignedInteger, Channel0, "LoadParameters",			//Data Type / Motor Channel / Name
		                    1.0, _CONFIG_, 1.0, eUnitPrefixONE, ePhysicalUnitNONE);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		SaveParameters.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                    _CMDGRP_MOTOR_CONFIN_, _MCMD_SAVE_CONFIG_,				 				//Command Group / Command
		                    eDataTypeSignedInteger, Channel0, "SaveParameters",			//Data Type / Motor Channel / Name
		                    1.0, _CONFIG_, 1.0, eUnitPrefixONE, ePhysicalUnitNONE);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		MotorPWMs.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		               _CMDGRP_MOTOR_CTRL_, _SET_ALL_PWM_, eDataTypeSignedInteger);				//Command Group / Command / Data Type

		MotorRPMs.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		               _CMDGRP_MOTOR_CTRL_, _SET_ALL_RPM_, eDataTypeSignedInteger);				//Command Group / Command / Data Type

		RobotVelocity.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                   _CMDGRP_MOTOR_CTRL_, 0x30, eDataTypeSignedInteger);				//Command Group / Command / Data Type
		//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

		IOPortConfiguration.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                         _CMDGRP_MOTOR_CONFIN_, _MCMD_IO_PORT_CONFIG_,				 				//Command Group / Command
		                         eDataTypeSignedInteger, Channel0, "IOPortConfiguration",			//Data Type / Motor Channel / Name
		                         0.0, 31.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		ClearAllAbsolutRotations.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                              _CMDGRP_MOTOR_CTRL_, _SET_CLEAR_ALL_TICKS_ABS_, eDataTypeSignedInteger);	 				//Command Group / Command

		/*	BumperOn.Init(
		 m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		 _CMDGRP_MOTOR_CTRL_, _SET_MODE_WITH_BUMPER_, SignedInteger);	 				//Command Group / Command
		 */

		DigitalOUT.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                _CMDGRP_MOTOR_CTRL_, _SET_DIGITAL_OUT_,				//Command Group / Command / Data Type
		                eDataTypeSignedInteger, Channel0, "DigitalIN",	//Data Type / Motor Channel / Name
		                0.0, 31.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		DigitalIN.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		               _CMDGRP_MOTOR_STATUSIN_, _SREG_DIGITAL_IN_,			 					//Command Group / Command
		               eDataTypeSignedInteger, Channel0, "DigitalIN",	//Data Type / Motor Channel / Name
		               0.0, 31.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		AnalogInput1.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                  _CMDGRP_MOTOR_STATUSIN_, _SREG_ANALOG1_IN_,			 					//Command Group / Command
		                  eDataTypeSignedInteger, Channel0, "AnalogInput1",	//Data Type / Motor Channel / Name
		                  111.0, 1024.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		AnalogInput2.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                  _CMDGRP_MOTOR_STATUSIN_, _SREG_ANALOG2_IN_,			 					//Command Group / Command
		                  eDataTypeSignedInteger, Channel0, "AnalogInput2",	//Data Type / Motor Channel / Name
		                  333.0, 1024.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		CChannel Channel(1);
		for (unsigned int i = 0; i < Channel0.getMaxChannels(); i++)
		{
			MotorPointer = new CMotor;
			Channel.set(i);
			MotorPointer->Init(m_pTrans, pNextRequestCommand, Channel);
			Motor.push_back(*MotorPointer);
			delete MotorPointer;
		}
		m_bInitialized = true;
		return true;
	}

}  // namespace VMC
