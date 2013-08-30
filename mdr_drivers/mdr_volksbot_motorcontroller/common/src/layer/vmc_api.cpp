//////////////////////////////////////////////////////////////////////////////
///  @file CvmcAPI.cpp
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////

#include "layer/vmc_api.h"

namespace VMC
{

	std::list<CError> VMC_Errors;
	BYTE bumperTilt = 0;

//////////////////////////////////////////////////////////////////////////////
///  @brief standard constructor
//////////////////////////////////////////////////////////////////////////////
	CVmcApi::CVmcApi()
	{
#define KENNER "VMC API Version 0.96c"

		m_VMC_API_Version = KENNER;  ///mit Digitalem/analogen IO/can

		NextRequestCommand = _SREG_MOT_RPM_;
		m_VMC.Init(&m_Trans, &NextRequestCommand);		//have to be first

		//// Defining buildup for response messages /////
		std::vector<CData*> DataPointers;
		CChannel Channel;
		unsigned int i;

		// Actual RPM
		for (i = 0; i < Channel.getMaxChannels(); i++)
		{
			DataPointers.push_back(&(m_VMC.Motor[i].ActualRPM));
		}
		m_Request.push_back(CRequest(_CMDGRP_MOTOR_STATUSOUT_, _SREG_MOT_RPM_, DataPointers, false));
		DataPointers.clear();

		// Actual PWM
		for (i = 0; i < Channel.getMaxChannels(); i++)
		{
			DataPointers.push_back(&(m_VMC.Motor[i].ActualPWM));
		}
		m_Request.push_back(CRequest(_CMDGRP_MOTOR_STATUSOUT_, _SREG_MOT_PWM_, DataPointers, false));
		DataPointers.clear();

		//Actual Current
		for (i = 0; i < Channel.getMaxChannels(); i++)
		{
			DataPointers.push_back(&(m_VMC.Motor[i].ActualCurrent));
		}
		m_Request.push_back(CRequest(_CMDGRP_MOTOR_STATUSOUT_, _SREG_MOT_CURRENT_, DataPointers, false));
		DataPointers.clear();

		//Absolut Tics(!)
		for (i = 0; i < Channel.getMaxChannels(); i++)
		{

			DataPointers.push_back(&(m_VMC.Motor[i].AbsolutRotations));
		}
		m_Request.push_back(CRequest(_CMDGRP_MOTOR_STATUSOUT_, _SREG_MOT_TICKS_ABS_, DataPointers, false));
		DataPointers.clear();

		//EncoderTicksRelativ
		for (i = 0; i < Channel.getMaxChannels(); i++)
		{
			DataPointers.push_back(&(m_VMC.Motor[i].EncoderTicksRelativ));
		}
		m_Request.push_back(CRequest(_CMDGRP_MOTOR_STATUSOUT_, _SREG_MOT_TICKS_REL_, DataPointers, false));
		DataPointers.clear();

		//ActualPowerOutput
		for (i = 0; i < Channel.getMaxChannels(); i++)
		{
			DataPointers.push_back(&(m_VMC.Motor[i].ActualPowerOutput));
		}
		m_Request.push_back(CRequest(_CMDGRP_MOTOR_STATUSOUT_, _SREG_MOT_POWER_, DataPointers, false));
		DataPointers.clear();

		//ActualWindingTemperature
		for (i = 0; i < Channel.getMaxChannels(); i++)
		{
			DataPointers.push_back(&(m_VMC.Motor[i].ActualWindingTemperature));
		}
		m_Request.push_back(CRequest(_CMDGRP_MOTOR_STATUSOUT_, _SREG_MOT_TEMP_, DataPointers, false));
		DataPointers.clear();

		//Battery Voltage
		DataPointers.push_back(&(m_VMC.BatteryVoltage));
		m_Request.push_back(CRequest(_CMDGRP_MOTOR_STATUSOUT_, _SREG_BATTERY_, DataPointers, false));
		DataPointers.clear();

		//Analog Inputs
		DataPointers.push_back(&(m_VMC.AnalogInput1));
		m_Request.push_back(CRequest(_CMDGRP_MOTOR_STATUSOUT_, _SREG_ANALOG1_IN_, DataPointers, false));
		DataPointers.clear();

		DataPointers.push_back(&(m_VMC.AnalogInput2));
		m_Request.push_back(CRequest(_CMDGRP_MOTOR_STATUSOUT_, _SREG_ANALOG2_IN_, DataPointers, false));
		DataPointers.clear();

		//Digital Inputs
		DataPointers.push_back(&(m_VMC.DigitalIN));
		m_Request.push_back(CRequest(_CMDGRP_MOTOR_STATUSOUT_, _SREG_DIGITAL_IN_, DataPointers, false));
		DataPointers.clear();

		m_RequestIterator = m_Request.begin();			// have to be last
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief selects a adapter
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////

	int CVmcApi::WhichBumper()
	{
		BYTE Help = bumperTilt;
		bumperTilt = 0;
		return Help;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief selects a adapter
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CVmcApi::selectHardwareAdapter(const EnumAdapter& Adapter)
	{

		return m_Trans.m_Com.selectAdapter(Adapter);
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief Open Device
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CVmcApi::openDevice()
	{
		return m_Trans.openDevice();
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief Close Device
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CVmcApi::closeDevice()
	{
		return m_Trans.closeDevice();
	}
//////////////////////////////////////////////////////////////////////////////
///  @brief sets a response Message active or inactive
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CVmcApi::configRequestMessage(unsigned int nRequestNo, bool bActive)
	{
		if (nRequestNo >= 0 && nRequestNo < m_Request.size())
		{
			m_Request[nRequestNo].setActiveSwitch(bActive);
			return true;
		}
		return false;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief sets a response Message active or inactive
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CVmcApi::printRequestMessage()
	{
		for (unsigned int i = 0; i < m_Request.size(); i++)
		{
			std::cout << m_Request[i];
		}
		return true;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief finds out which request command is the next one
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CVmcApi::getNextRequestCommand()
	{

		for (unsigned int i = 0; i < m_Request.size(); i++)
		{

			m_RequestIterator++;

			if (m_RequestIterator == m_Request.end())
			{
				m_RequestIterator = m_Request.begin();
			}

			if (m_RequestIterator->getActiveSwitchState() == true)
			{			// request command have to be active

				NextRequestCommand = m_RequestIterator->getCommand();
				return true;
			}
		}
		return false;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief receives messages from the VMC and stores the data
///  @returns true = OK / false = Error A
//////////////////////////////////////////////////////////////////////////////
	CStorage& CVmcApi::useVMC()
	{

//@@@@@@@@@
//	printf("CStorage& CVmcApi::useVMC() {\n");

		m_Trans.receiveMessage();						// receive data from interface

		while (m_Trans.findMessage(m_ReceivedMessage) == true)
		{  // finds a message in received data
			///////////////////////////////////DEBUG///////////////////////////////////////////
#ifdef VMC_DEBUG
			std::cout << " received " << m_ReceivedMessage << std::endl;  //debug only
#endif
			///////////////////////////////////DEBUG///////////////////////////////////////////

			if (m_ReceivedMessage.verifyCRC() == false)
			{			// verifys CRC from received message
				VMC_Errors.push_back(CError("CRC mismatch", "VMC_API::useVMC()", m_ReceivedMessage));
			}
			else
			{
				storeMessageData(m_ReceivedMessage);				// stors data form received message
			}
		}
		/*
		 ///////////////////////////////////DEBUG///////////////////////////////////////////
		 #ifdef VMC_DEBUG
		 if(!_ReceivedMessage.isValidMessage())
		 VMC_Errors.push_back( CError("no message received", "CStorage CVmcApi::useVMC()") );
		 #endif
		 ///////////////////////////////////DEBUG///////////////////////////////////////////
		 */
		return m_VMC;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief stores the data from a received message
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CVmcApi::storeMessageData(const CMessage& Message)
	{

		switch (Message.getCommandGroup())
		{
			case _CMDGRP_BASE_:
				return (interpreteBasicSystemCommands(Message));
				break;
			case _CMDGRP_MOTOR_CONFOUT_:
				return (interpreteConfigurationResponse(Message));
				break;
			case _CMDGRP_MOTOR_STATUSOUT_:
				return (interpreteStatusResponse(Message));
				break;
			case _CMDGRP_CONTROLLER_PAROUT_:
				return (interpreteControllerConfigurationResponse(Message));
				break;
			case _CMDGRP_MOTOR_ERR_:
			case 0x11:
				return (interpreteErrorMessages(Message));
				break;

			default:
				VMC_Errors.push_back(CError("unknown command group", "VMC_API::StoreMessageData()", Message));
				break;
		}
		return false;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief Interprete the Basic System Commands (Command Group 0x11)
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CVmcApi::interpreteBasicSystemCommands(const CMessage& Message)
	{
		std::vector<BYTE> DataFrame;

		Message.getDataFrame(DataFrame);

		switch (Message.getCommand())
		{
			case 0x10:
				return (m_VMC.VMCVersion.setValue(DataFrame));
				break;

		}

		return true;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief Interprete the Configuration Response Message (Command Group 0x51)
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CVmcApi::interpreteConfigurationResponse(const CMessage& Message)
	{
		CChannel Channel;
		std::vector<BYTE> DataFrame;

		Message.getDataFrame(DataFrame);

		/// gets motor channel from data frame
		if (DataFrame.size() > 2)
		{			//at least two bytes
			Channel = DataFrame[0] - 1;			//get motor channel number
			DataFrame.erase(DataFrame.begin(), DataFrame.begin() + 1);  //delete channel number from data frame
		}
		else
		{
			VMC_Errors.push_back(CError("no data to store", "VMC_API::InterpreteConfigurationResponse()", Message));
			return false;
		}

		if (Message.getCommand() != 0x50 && Message.getCommand() != 0x51)
		{
			if (!Channel.validChannel())
			{
				VMC_Errors.push_back(CError("invalid motor channel", "VMC_API::InterpreteConfigurationResponse()", Message));
				return false;
			}
		}

		switch (Message.getCommand())
		{
			case _MCMD_SAVE_CONFIG_:
				return (m_VMC.SaveParameters.setValue(DataFrame));
				break;
			case _MCMD_LOAD_CONFIG_:
				return (m_VMC.LoadParameters.setValue(DataFrame));
				break;

			case _MCMD_CONTROLLER_:
				return (m_VMC.Motor[Channel.get()].VelocityControllerState.setValue(DataFrame));
				break;

			case _MCMD_USE_PWM_:
				return (m_VMC.Motor[Channel.get()].VelocityInputType.setValue(DataFrame));
				break;
			case _MCMD_LIMITER_:
				return (m_VMC.Motor[Channel.get()].CurrentControllerState.setValue(DataFrame));
				break;
			case _MCMD_TIMEOUT_:
				return (m_VMC.VMCTimeout.setValue(DataFrame));
				break;

			case _MCMD_MAX_CUR_:
				return (m_VMC.Motor[Channel.get()].MaximumCurrent.setValue(DataFrame));
				break;
			case _MCMD_MAX_RPM_:
				return (m_VMC.Motor[Channel.get()].MaximumRPM.setValue(DataFrame));
				break;
			case _MCMD_NOM_CUR_:
				return (m_VMC.Motor[Channel.get()].NominalCurrent.setValue(DataFrame));
				break;
			case _MCMD_NOM_RPM_:
				return (m_VMC.Motor[Channel.get()].NominalRPM.setValue(DataFrame));
				break;

			case _MCMD_GEAR_RATIO_:
				return (m_VMC.Motor[Channel.get()].GearRatio.setValue(DataFrame));
				break;
			case _MCMD_WHEEL_RADIUS_:
				return (m_VMC.Motor[Channel.get()].WheelRadius.setValue(DataFrame));
				break;
//		case _MCMD_AXE_LENGTH_:
			case _MCMD_TICKS_ROT_:
				return (m_VMC.Motor[Channel.get()].EncoderTicks.setValue(DataFrame));
				break;
			case _MCMD_SPEC_TORQUE_:
				return (m_VMC.Motor[Channel.get()].TorqueConstant.setValue(DataFrame));
				break;
			case _MCMD_DIRECTION_:
				return (m_VMC.Motor[Channel.get()].MotorState.setValue(DataFrame));
				break;

			case _MCMD_MAX_TEMP_:
				return (m_VMC.Motor[Channel.get()].MaximumTemperature.setValue(DataFrame));
				break;
			case _MCMD_NOM_TEMP_:
				return (m_VMC.Motor[Channel.get()].NominalTemperature.setValue(DataFrame));
				break;
			case _MCMD_ENV_TEMP_:
				return (m_VMC.Motor[Channel.get()].AmbientTemperature.setValue(DataFrame));
				break;
			case _MCMD_WINDING_T_:
				return (m_VMC.Motor[Channel.get()].ThermalTimeConstantWinding.setValue(DataFrame));
				break;
			case _MCMD_WINDING_GN_:
				return (m_VMC.Motor[Channel.get()].ThermalResistanceWindingDenominator.setValue(DataFrame));
				break;
			case _MCMD_WINDING_GZ_:
				return (m_VMC.Motor[Channel.get()].ThermalResistanceWindingNumerator.setValue(DataFrame));
				break;
			case _MCMD_CHASSIS_T_:
				return (m_VMC.Motor[Channel.get()].ThermalTimeConstantMotor.setValue(DataFrame));
				break;
			case _MCMD_CHASSIS_GN_:
				return (m_VMC.Motor[Channel.get()].ThermalResistanceHousingDenominator.setValue(DataFrame));
				break;
			case _MCMD_CHASSIS_GZ_:
				return (m_VMC.Motor[Channel.get()].ThermalResistanceHousingNumerator.setValue(DataFrame));
				break;

			case _MCMD_CLEAR_ONE_TICKS_ABS_:
				return (m_VMC.Motor[Channel.get()].ClearOneAbsolutRotations.setValue(DataFrame));
				break;

			case _MCMD_IO_PORT_CONFIG_:
			{

//			const double Help = 0x1f;

				//			printf("at setValue Cvmc\n");
				return (m_VMC.IOPortConfiguration.setValue(DataFrame));
//			return( m_VMC.IOPortConfiguration.Set(Help) );

			}

				break;

//		case _MCMD_CLEAR_ALL_TICKS_ABS_: 	return( m_VMC.Motor[Channel.get()].ClearAbsolutRotations.setValue(DataFrame) ); break;

			default:
				VMC_Errors.push_back(CError("unknown command", "VMC_API::InterpreteConfigurationResponse()", Message));
				break;
		}

		return false;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief Interprete the Status Response Message (Command Group 0x55)
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CVmcApi::interpreteStatusResponse(const CMessage& Message)
	{
		CChannel Channel;
		std::vector<BYTE> DataFrame;
		std::vector<BYTE> Value;
		CData* DataPointer;
		CRequest actualRequest;
		CChannel FirstByte;

		for (unsigned int u = 0; u < m_Request.size(); u++)
		{			// finds request buildup which fits the received one
			if (Message.getCommand() == m_Request[u].getCommand())
			{
				actualRequest = m_Request[u];
				break;
			}
		}

		if (actualRequest.getCommand() == 0)
		{
			VMC_Errors.push_back(CError("unknown command", "VMC_API::InterpreteStatusResponse()", Message));
			return false;
		}

		Message.getDataFrame(DataFrame);
		/// gets motor channel from data frame
		if (DataFrame.size() > 2)
		{			//at least two bytes
			Channel = DataFrame[0] - 1;			//get motor channel number
			FirstByte = DataFrame[0];
			DataFrame.erase(DataFrame.begin(), DataFrame.begin() + 1);  //delete channel number from data frame
		}
		else
		{
			VMC_Errors.push_back(CError("no data to store", "VMC_API::InterpreteStatusResponse()", Message));
			return false;
		}

		if (Channel.get() >= Channel.getMaxChannels())
		{
			VMC_Errors.push_back(CError("invalid motor channel", "VMC_API::InterpreteStatusResponse()", Message));
			return false;
		}

		unsigned int buildupSize;
		if (FirstByte == 0)
			buildupSize = static_cast<unsigned int>(actualRequest.getResponseBuildupSize());
		else
			buildupSize = Channel.get() + 1;

		for (unsigned int i = Channel.get(); i < buildupSize; i++)
		{

			DataPointer = actualRequest.getResponseBuildup(i);

			switch (DataPointer->getDataType())
			{
				case eDataTypeUnsignedChar:
					if (DataFrame.size() < 1)
					{
						VMC_Errors.push_back(CError("invalid data length", "VMC_API::InterpreteStatusResponse()", Message));
						return false;
					}
					Value.push_back(DataFrame[0]);
					DataFrame.erase(DataFrame.begin(), DataFrame.begin() + 1);
					break;
				case eDataTypeSignedChar:
					if (DataFrame.size() < 1)
					{
						VMC_Errors.push_back(CError("invalid data length", "VMC_API::InterpreteStatusResponse()", Message));
						return false;
					}
					Value.push_back(DataFrame[0]);
					DataFrame.erase(DataFrame.begin(), DataFrame.begin() + 1);
					break;
				case eDataTypeUnsignedInteger:
					if (DataFrame.size() < 2)
					{
						VMC_Errors.push_back(CError("invalid data length", "VMC_API::InterpreteStatusResponse()", Message));
						return false;
					}
					Value.push_back(DataFrame[0]);
					Value.push_back(DataFrame[1]);
					DataFrame.erase(DataFrame.begin(), DataFrame.begin() + 2);
					break;
				case eDataTypeSignedInteger:
					if (DataFrame.size() < 2)
					{
						VMC_Errors.push_back(CError("invalid data length", "VMC_API::InterpreteStatusResponse()", Message));
						return false;
					}
					Value.push_back(DataFrame[0]);
					Value.push_back(DataFrame[1]);
					DataFrame.erase(DataFrame.begin(), DataFrame.begin() + 2);

					break;
				case eDataTypeSignedLong:
					if (DataFrame.size() < 4)
					{
						VMC_Errors.push_back(CError("invalid data length", "VMC_API::InterpreteStatusResponse()", Message));
						return false;
					}
					Value.push_back(DataFrame[0]);
					Value.push_back(DataFrame[1]);
					Value.push_back(DataFrame[2]);
					Value.push_back(DataFrame[3]);
					DataFrame.erase(DataFrame.begin(), DataFrame.begin() + 4);
					break;
				case eDataTypeString:
					VMC_Errors.push_back(CError("string not implemented yet", "VMC_API::InterpreteStatusResponse()", Message));
					return false;
					break;
			}

			DataPointer->setValue(Value);
			Value.clear();
		}
		DataPointer = NULL;

		getNextRequestCommand();

		return true;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief Interprete the Controller Configuration Response Message (Command Group 0x56)
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CVmcApi::interpreteControllerConfigurationResponse(const CMessage& Message)
	{
		CChannel Channel;
		std::vector<BYTE> DataFrame;

		Message.getDataFrame(DataFrame);
		/// gets motor channel from data frame
		if (DataFrame.size() > 2)
		{			//at least two bytes
			Channel = DataFrame[0] - 1;			//get motor channel number
			DataFrame.erase(DataFrame.begin(), DataFrame.begin() + 1);  //delete channel number from data frame
		}
		else
		{
			VMC_Errors.push_back(CError("no data to store", "VMC_API::InterpreteConfigurationResponse()", Message));
			return false;
		}

		if (!Channel.validChannel())
		{
			VMC_Errors.push_back(CError("invalid motor channel", "VMC_API::InterpreteConfigurationResponse()", Message));
			return false;
		}

		switch (Message.getCommand())
		{
			case _CTRL_PID_KP_:
				return (m_VMC.Motor[Channel.get()].VelocityControllerLinearPart.setValue(DataFrame));
				break;
			case _CTRL_PID_TV_:
				return (m_VMC.Motor[Channel.get()].VelocityControllerDifferentialPart.setValue(DataFrame));
				break;
			case _CTRL_PID_TN_:
				return (m_VMC.Motor[Channel.get()].VelocityControllerIntegralPart.setValue(DataFrame));
				break;
			case _CTRL_PRAMP_:
				return (m_VMC.Motor[Channel.get()].PositivRamp.setValue(DataFrame));
				break;
			case _CTRL_NRAMP_:
				return (m_VMC.Motor[Channel.get()].NegativRamp.setValue(DataFrame));
				break;
			case _CTRL_DEADBAND_:
				return (m_VMC.Motor[Channel.get()].DeadBand.setValue(DataFrame));
				break;

				//...				// still to do

			default:
				VMC_Errors.push_back(CError("unknown command", "VMC_API::InterpreteControllerConfigurationResponse()", Message));
				break;
		}

		return false;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief Interprete the Error Messages (Command Group 0x59)
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CVmcApi::interpreteErrorMessages(const CMessage& Message)
	{
		std::vector<BYTE> DataFrame;
		std::stringstream ErrorString;

		Message.getDataFrame(DataFrame);

		if (Message.getCommand() == _BUMPER_ERR_)
		{
			bumperTilt = (BYTE) DataFrame[2];
			return true;
		}

		for (unsigned int i = 0; i < DataFrame.size(); i++)
		{		// stors VMC error string to a proper string
			ErrorString << DataFrame[i];
		}

		switch (Message.getCommand())
		{
			case _UNKNOWN_ERR_:
				VMC_Errors.push_back(CError("VMC Error: unknown", "VMC_API::InterpreteErrorMessages()", ErrorString.str(), Message));
				break;
			case _PARAM_ERR_:
				VMC_Errors.push_back(CError("VMC Error: parameter", "VMC_API::InterpreteErrorMessages()", ErrorString.str(), Message));
				break;
			case _RANGE_ERR_:
				VMC_Errors.push_back(CError("VMC Error: parameter out of range", "VMC_API::InterpreteErrorMessages()", ErrorString.str(), Message));
				break;

			default:
				VMC_Errors.push_back(CError("VMC Error: unknown", "VMC_API::InterpreteErrorMessages()", ErrorString.str(), Message));
				break;
		}

		return true;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief Prints all errors to the standard output
//////////////////////////////////////////////////////////////////////////////
	void CVmcApi::printAllErrors()
	{

		if (VMC_Errors.size() == 0)
		{
			std::cout << "No Errors" << std::endl;
			return;
		}

		std::list<CError>::iterator ErrorIterator = VMC_Errors.begin();

		while (ErrorIterator != VMC_Errors.end())
		{
			std::cout << *ErrorIterator << std::endl;

			ErrorIterator++;
		}
		VMC_Errors.clear();  //@@@@@
	}

}  // namespace VMC

