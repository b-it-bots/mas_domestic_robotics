//////////////////////////////////////////////////////////////////////////////
///  @file CMotor.cpp
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////

#include "storage/motor.h"

namespace VMC
{

//////////////////////////////////////////////////////////////////////////////
///  @brief Initialized the all parameters and runtime variables
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CMotor::Init(CTranslationLayer* Trans, BYTE* pNextRequestCommand, CChannel ch)
	{

		if (m_bInitialized)
			return false;

		m_pTrans = Trans;
		m_Channel = ch;

		/*=========================================== CONFIGURATION PARAMETER =================================================*/

		/*==================VelocityController=====================*/
		VelocityControllerState.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                             _CMDGRP_MOTOR_CONFIN_, _MCMD_CONTROLLER_,								//Command Group / Command
		                             eDataTypeSignedInteger, m_Channel, "VelocityControllerState",	//Data Type / Motor Channel / Name
		                             0.0, 1.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		VelocityControllerLinearPart.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                                  _CMDGRP_CONTROLLER_PARIN_, _CTRL_PID_KP_,								//Command Group / Command
		                                  eDataTypeSignedInteger, m_Channel, "VelocityControllerLinearPart",	//Data Type / Motor Channel / Name
		                                  0.0, 32767.0, -32767.0, eUnitPrefixONE, ePhysicalUnitNONE);//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		VelocityControllerDifferentialPart.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                                        _CMDGRP_CONTROLLER_PARIN_, _CTRL_PID_TV_,								//Command Group / Command
		                                        eDataTypeSignedInteger, m_Channel, "VelocityControllerDifferentialPart",	//Data Type / Motor Channel / Name
		                                        0.0, 32767.0, -32767.0, eUnitPrefixONE, ePhysicalUnitNONE);	//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		VelocityControllerIntegralPart.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                                    _CMDGRP_CONTROLLER_PARIN_, _CTRL_PID_TN_,								//Command Group / Command
		                                    eDataTypeSignedInteger, m_Channel, "VelocityControllerIntegralPart",	//Data Type / Motor Channel / Name
		                                    0.0, 32767.0, -32767.0, eUnitPrefixONE, ePhysicalUnitNONE);	//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		PositivRamp.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                 _CMDGRP_CONTROLLER_PARIN_, _CTRL_PRAMP_,								//Command Group / Command
		                 eDataTypeSignedInteger, m_Channel, "PositiveRamp",	//Data Type / Motor Channel / Name
		                 0.0, 32767.0, -32767.0, eUnitPrefixONE, ePhysicalUnitNONE);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		NegativRamp.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                 _CMDGRP_CONTROLLER_PARIN_, _CTRL_NRAMP_,								//Command Group / Command
		                 eDataTypeSignedInteger, m_Channel, "NegativeRamp",	//Data Type / Motor Channel / Name
		                 0.0, 32767.0, -32767.0, eUnitPrefixONE, ePhysicalUnitNONE);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		DeadBand.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		              _CMDGRP_CONTROLLER_PARIN_, _CTRL_DEADBAND_,								//Command Group / Command
		              eDataTypeSignedInteger, m_Channel, "DeadBand",	//Data Type / Motor Channel / Name
		              0.0, 32767.0, -32767.0, eUnitPrefixONE, ePhysicalUnitNONE);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		/*==============================================*/

		/*===================CurrentController=======================*/
		CurrentControllerState.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                            _CMDGRP_MOTOR_CONFIN_, _MCMD_LIMITER_,								//Command Group / Command
		                            eDataTypeSignedInteger, m_Channel, "CurrentControllerState",	//Data Type / Motor Channel / Name
		                            0.0, 1.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);			//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		CurrentControllerLinearPart.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                                 _CMDGRP_CONTROLLER_PARIN_, 0x0,								//Command Group / Command
		                                 eDataTypeSignedInteger, m_Channel, "CurrentControllerLinearPart",	//Data Type / Motor Channel / Name
		                                 0.0, 32767.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		CurrentControllerDifferentialPart.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                                       _CMDGRP_CONTROLLER_PARIN_, 0x0,								//Command Group / Command
		                                       eDataTypeSignedInteger, m_Channel, "CurrentControllerDifferentialPart",  //Data Type / Motor Channel / Name
		                                       0.0, 32767.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		CurrentControllerIntegralPart.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                                   _CMDGRP_CONTROLLER_PARIN_, 0x0,								//Command Group / Command
		                                   eDataTypeSignedInteger, m_Channel, "CurrentControllerIntegralPart",  //Data Type / Motor Channel / Name
		                                   0.0, 32767.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);	//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		                                   /*==============================================*/
//12345

		VelocityInputType.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                       _CMDGRP_MOTOR_CONFIN_, _MCMD_USE_PWM_,			 					//Command Group / Command
		                       eDataTypeSignedInteger, m_Channel, "VelocityInputType",			//Data Type / Motor Channel / Name
		                       0.0, 1.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		/*====================Current==========================*/
		MaximumCurrent.Init(m_pTrans, pNextRequestCommand,				//Pointer to the Translation Layer / Pointer to next Request Command
		                    _CMDGRP_MOTOR_CONFIN_, _MCMD_MAX_CUR_,									//Command Group / Command
		                    eDataTypeSignedInteger, m_Channel, "MaximumCurrent",	//Data Type / Motor Channel / Name
		                    0.0, _MAX_CUR_, 0.0, eUnitPrefixMILLI, ePhysicalUnitAMPERE);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		NominalCurrent.Init(m_pTrans, pNextRequestCommand,				//Pointer to the Translation Layer / Pointer to next Request Command
		                    _CMDGRP_MOTOR_CONFIN_, _MCMD_NOM_CUR_,									//Command Group / Command
		                    eDataTypeSignedInteger, m_Channel, "NominalCurrent",	//Data Type / Motor Channel / Name
		                    0.0, _NOM_CUR_, 0.0, eUnitPrefixMILLI, ePhysicalUnitAMPERE);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		                    /*==============================================*/

		/*=====================Hardware=========================*/
		WheelRadius.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                 _CMDGRP_MOTOR_CONFIN_, _MCMD_WHEEL_RADIUS_,								//Command Group / Command
		                 eDataTypeSignedInteger, m_Channel, "WheelRadius",	//Data Type / Motor Channel / Name
		                 0.0, _WHEEL_RADIUS_, 0.0, eUnitPrefixMILLI, ePhysicalUnitMETER);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		GearRatio.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		               _CMDGRP_MOTOR_CONFIN_, _MCMD_GEAR_RATIO_, 							//Command Group / Command
		               eDataTypeSignedInteger, m_Channel, "GearRatio",  //Data Type / Motor Channel / Name
		               0.0, _GEAR_RATIO_, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);			//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		AxeLength.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		               _CMDGRP_MOTOR_CONFIN_, _MCMD_AXE_LENGTH_,								//Command Group / Command
		               eDataTypeSignedInteger, m_Channel, "AxeLength",  //Data Type / Motor Channel / Name
		               0.0, _AXE_LENGTH_, 0.0, eUnitPrefixMILLI, ePhysicalUnitMETER);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		EncoderTicks.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                  _CMDGRP_MOTOR_CONFIN_, _MCMD_TICKS_ROT_,								//Command Group / Command
		                  eDataTypeSignedInteger, m_Channel, "EncoderTicks",	//Data Type / Motor Channel / Name
		                  0.0, _TICKS_ROT_, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);			//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		                  /*==============================================*/

		/*===================Torque=======================*/
		TorqueConstant.Init(m_pTrans, pNextRequestCommand,				//Pointer to the Translation Layer / Pointer to next Request Command
		                    _CMDGRP_MOTOR_CONFIN_, _MCMD_SPEC_TORQUE_, 								//Command Group / Command
		                    eDataTypeSignedInteger, m_Channel, "TorqueConstant",	//Data Type / Motor Channel / Name
		                    0.0, _SPEC_TORQUE_, 0.0, eUnitPrefixMILLI, ePhysicalUnitNEWTONMETER_AMPERE);//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		                    /*==========================================*/

		/*===================Temperature=======================*/
		ThermalResistanceHousingNumerator.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                                       _CMDGRP_MOTOR_CONFIN_, _MCMD_CHASSIS_GZ_,								//Command Group / Command
		                                       eDataTypeSignedInteger, m_Channel, "ThermalResistanceHousingNumerator",  //Data Type / Motor Channel / Name
		                                       0.0, _CHASSIS_GZ_, 0.0, eUnitPrefixMILLI, ePhysicalUnitSECOND);	//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		ThermalResistanceHousingDenominator.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                                         _CMDGRP_MOTOR_CONFIN_, _MCMD_CHASSIS_GN_,					//Command Group / Command
		                                         eDataTypeSignedInteger, m_Channel, "ThermalResistanceHousingDenominator",	//Data Type / Motor Channel / Name
		                                         0.0, _CHASSIS_GN_, 0.0, eUnitPrefixMILLI, ePhysicalUnitSECOND);//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		ThermalResistanceWindingNumerator.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                                       _CMDGRP_MOTOR_CONFIN_, _MCMD_WINDING_GZ_,								//Command Group / Command
		                                       eDataTypeSignedInteger, m_Channel, "ThermalResistanceWindingNumerator",  //Data Type / Motor Channel / Name
		                                       0.0, _WINDING_GZ_, 0.0, eUnitPrefixMILLI, ePhysicalUnitSECOND);	//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		ThermalResistanceWindingDenominator.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                                         _CMDGRP_MOTOR_CONFIN_, _MCMD_WINDING_GN_,								//Command Group / Command
		                                         eDataTypeSignedInteger, m_Channel, "ThermalResistanceWindingDenominator",	//Data Type / Motor Channel / Name
		                                         0.0, _WINDING_GN_, 0.0, eUnitPrefixMILLI, ePhysicalUnitSECOND);//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		ThermalTimeConstantWinding.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                                _CMDGRP_MOTOR_CONFIN_, _MCMD_WINDING_T_,								//Command Group / Command
		                                eDataTypeSignedInteger, m_Channel, "ThermalTimeConstantWinding",	//Data Type / Motor Channel / Name
		                                0.0, _WINDING_T_, 0.0, eUnitPrefixMILLI, ePhysicalUnitSECOND);	//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		ThermalTimeConstantMotor.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                              _CMDGRP_MOTOR_CONFIN_, _MCMD_CHASSIS_T_,								//Command Group / Command
		                              eDataTypeSignedInteger, m_Channel, "ThermalTimeConstantMotor",	//Data Type / Motor Channel / Name
		                              0.0, _CHASSIS_T_, 0.0, eUnitPrefixMILLI, ePhysicalUnitSECOND);//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		MaximumTemperature.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                        _CMDGRP_MOTOR_CONFIN_, _MCMD_MAX_TEMP_,								//Command Group / Command
		                        eDataTypeSignedInteger, m_Channel, "MaximumTemperature",	//Data Type / Motor Channel / Name
		                        0.0, _MAX_TEMP_, 0.0, eUnitPrefixONE, ePhysicalUnitDEGREECELSIUS);	//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		NominalTemperature.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                        _CMDGRP_MOTOR_CONFIN_, _MCMD_NOM_TEMP_,								//Command Group / Command
		                        eDataTypeSignedInteger, m_Channel, "NominalTemperature",	//Data Type / Motor Channel / Name
		                        0.0, _NOM_TEMP_, 0.0, eUnitPrefixONE, ePhysicalUnitDEGREECELSIUS);	//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		AmbientTemperature.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                        _CMDGRP_MOTOR_CONFIN_, _MCMD_ENV_TEMP_,								//Command Group / Command
		                        eDataTypeSignedInteger, m_Channel, "AmbientTemperature",	//Data Type / Motor Channel / Name
		                        0.0, _ENV_TEMP_, 0.0, eUnitPrefixONE, ePhysicalUnitDEGREECELSIUS);	//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		                        /*==============================================*/

		/*=======================RPM=======================*/
		NominalRPM.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                _CMDGRP_MOTOR_CONFIN_, _MCMD_NOM_RPM_,								//Command Group / Command
		                eDataTypeSignedInteger, m_Channel, "NominalRPM",	//Data Type / Motor Channel / Name
		                0.0, _NOM_RPM_, 0.0, eUnitPrefixONE, ePhysicalUnitRPM);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		MaximumRPM.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                _CMDGRP_MOTOR_CONFIN_, _MCMD_MAX_RPM_,								//Command Group / Command
		                eDataTypeSignedInteger, m_Channel, "MaximumRPM",	//Data Type / Motor Channel / Name
		                0.0, _MAX_RPM_, 0.0, eUnitPrefixONE, ePhysicalUnitRPM);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		                /*==============================================*/

		/*=======================PWM=======================*/
		NominalPWM.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                0x0, 0x0,								//Command Group / Command
		                eDataTypeSignedInteger, m_Channel, "NominalPWM",	//Data Type / Motor Channel / Name
		                0.0, 32767.0, -32767.0, eUnitPrefixONE, ePhysicalUnitNONE);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		MaximumPWM.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                0x0, 0x0,								//Command Group / Command
		                eDataTypeSignedInteger, m_Channel, "MaximumPWM",	//Data Type / Motor Channel / Name
		                0.0, 32767.0, -32767.0, eUnitPrefixONE, ePhysicalUnitNONE);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		                /*==============================================*/
		/*=========================================== END CONFIGURATION PARAMETER =================================================*/

		/*=========================================== RUNTIME VARIABLES =================================================*/
		/*====================Current==========================*/
		ActualCurrent.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                   _CMDGRP_MOTOR_STATUSIN_, _SREG_MOT_CURRENT_,			 					//Command Group / Command
		                   eDataTypeSignedInteger, m_Channel, "ActualCurrent",  //Data Type / Motor Channel / Name
		                   0.0, 32767.0, -32767.0, eUnitPrefixMILLI, ePhysicalUnitAMPERE);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		ActualPowerOutput.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                       _CMDGRP_MOTOR_STATUSIN_, _SREG_MOT_POWER_,								//Command Group / Command
		                       eDataTypeSignedInteger, m_Channel, "ActualPowerOutput",  //Data Type / Motor Channel / Name
		                       0.0, 32767.0, -32767.0, eUnitPrefixONE, ePhysicalUnitWATT);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		                       /*===================Odometrie=======================*/
		AbsolutRotations.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                      _CMDGRP_MOTOR_STATUSIN_, _SREG_MOT_TICKS_ABS_,								//Command Group / Command
		                      eDataTypeSignedLong, m_Channel, "AbsolutRotations",  //Data Type / Motor Channel / Name
		                      0.0, 2147483647.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);			//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		ClearOneAbsolutRotations.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                              _CMDGRP_MOTOR_CONFIN_, _MCMD_CLEAR_ONE_TICKS_ABS_,								//Command Group / Command
		                              eDataTypeSignedInteger, m_Channel, "ClearOneAbsolutRotations",	//Data Type / Motor Channel / Name
		                              0.0, 1.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);			//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		EncoderTicksRelativ.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                         _CMDGRP_MOTOR_STATUSIN_, _SREG_MOT_TICKS_REL_,								//Command Group / Command
		                         eDataTypeSignedInteger, m_Channel, "EncoderTicksRelativ",	//Data Type / Motor Channel / Name
		                         0.0, 32767.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);			//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		                         /*==============================================*/

		/*===================Temperature=======================*/
		ActualWindingTemperature.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                              _CMDGRP_MOTOR_STATUSIN_, _SREG_MOT_TEMP_,								//Command Group / Command
		                              eDataTypeSignedInteger, m_Channel, "ActualWindingTemperature",	//Data Type / Motor Channel / Name
		                              0.0, 32767.0, -200.0, eUnitPrefixONE, ePhysicalUnitDEGREECELSIUS);//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		ActualChassisTemperature.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                              _CMDGRP_MOTOR_STATUSIN_, 0x0,			 					//Command Group / Command
		                              eDataTypeSignedInteger, m_Channel, "ActualChassisTemperature",	//Data Type / Motor Channel / Name
		                              0.0, 32767.0, -200.0, eUnitPrefixONE, ePhysicalUnitDEGREECELSIUS);//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		                              /*==============================================*/

		/*=======================RPM=======================*/
		DesiredRPM.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                _CMDGRP_MOTOR_CTRL_, _SET_ONE_RPM_,								//Command Group / Command
		                eDataTypeSignedInteger, m_Channel, "DesiredRPM",	//Data Type / Motor Channel / Name
		                0.0, 32767.0, -32767.0, eUnitPrefixONE, ePhysicalUnitRPM);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		ActualRPM.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		               _CMDGRP_MOTOR_STATUSIN_, _SREG_MOT_RPM_,								//Command Group / Command
		               eDataTypeSignedInteger, m_Channel, "ActualRPM",  //Data Type / Motor Channel / Name
		               0.0, 32767.0, -32767.0, eUnitPrefixONE, ePhysicalUnitRPM);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		               /*==============================================*/

		/*=======================PWM=======================*/
		DesiredPWM.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                _CMDGRP_MOTOR_CTRL_, _SET_ONE_PWM_,								//Command Group / Command
		                eDataTypeSignedInteger, m_Channel, "DesiredPWM",	//Data Type / Motor Channel / Name
		                0.0, 32767.0, -32767.0, eUnitPrefixONE, ePhysicalUnitNONE);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		ActualPWM.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		               _CMDGRP_MOTOR_STATUSIN_, _SREG_MOT_PWM_,								//Command Group / Command
		               eDataTypeSignedInteger, m_Channel, "ActualPWM",  //Data Type / Motor Channel / Name
		               0.0, 32767.0, -32767.0, eUnitPrefixONE, ePhysicalUnitNONE);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		               /*==============================================*/

		/*=======================FALSE=======================*/
		Encoderfalse.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                  0x0, 0x0,								//Command Group / Command
		                  eDataTypeSignedInteger, m_Channel, "Encoderfalse",	//Data Type / Motor Channel / Name
		                  0.0, 1.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		MotorOberheat.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                   0x0, 0x0,								//Command Group / Command
		                   eDataTypeSignedInteger, m_Channel, "MotorOberheat",  //Data Type / Motor Channel / Name
		                   0.0, 1.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

		BridgeOverheat.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                    0x0, 0x0,								//Command Group / Command
		                    eDataTypeSignedInteger, m_Channel, "BridgeOverheat",	//Data Type / Motor Channel / Name
		                    0.0, 1.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		                    /*==============================================*/

		MotorState.Init(m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		                _CMDGRP_MOTOR_CONFIN_, _MCMD_DIRECTION_,			 					//Command Group / Command
		                eDataTypeSignedInteger, m_Channel, "MotorState",	//Data Type / Motor Channel / Name
		                0.0, 4.0, 0.0, eUnitPrefixONE, ePhysicalUnitNONE);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		                /*==============================================*/

		/*=========================================== END RUNTIME VARIABLES =================================================*/

		m_bInitialized = true;
		return true;
	}

}  // namespace VMC
