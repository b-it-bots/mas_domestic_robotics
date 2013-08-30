//////////////////////////////////////////////////////////////////////////////
///  @file CData.cpp
///  @author Jan Paulus
///  @version 0.1
///  @date 16.10.2006
//////////////////////////////////////////////////////////////////////////////

#include <cstdio>
#include "support/data.h"

namespace VMC
{

	bool CData::giveInitState()
	{
		return m_bInitialized;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief Initialized the CData Object
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CData::Init(CTranslationLayer* Trans, BYTE* pNextRequestCommand, const BYTE nCommandGroup, const BYTE nCommand, const EnumDataType DataType,
	                 const CChannel& MotorChannel, const std::string& cName, const double dValue, const double dMaximum, const double dMinimum,
	                 const EnumUnitPrefix UnitPrefix, const EnumPhysicalUnit Unit)
	{

		if (m_bInitialized)
			return false;

		m_pTrans = Trans;
		m_pNextRequestCommand = pNextRequestCommand;
		m_CommandGroup = nCommandGroup;
		m_Command = nCommand;
		m_DataType = DataType;
		m_MotorChannel = MotorChannel;
		m_sName = cName;

		if (dMaximum < dMinimum || dMaximum < dValue || dMinimum > dValue)
		{

			m_bInitialized = false;
			VMC_Errors.push_back(CError("Maximum in not bigger than Minimum or Value is not in range", "CData::Init()", m_sName));
			return false;
		}
		m_dValue = dValue;
		m_dMaximum = dMaximum;
		m_dMinimum = dMinimum;
		m_UnitPrefix = UnitPrefix;
		m_Unit = Unit;

		m_Timestamp.update();
		m_bInitialized = true;
		return true;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief sets the value (not to the VMC)
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CData::setValue(const double Value)
	{
///@@@@@@@@@@@
///	printf("setValue(const double Value)<%08x>\n", Value);

		if (!m_bInitialized)
			return false;

		if (m_dMaximum >= Value || m_dMinimum <= Value)
		{
			m_dValue = Value;
			m_Timestamp.update();
			return true;
		}
		VMC_Errors.push_back(CError("value out of range", "CData::setValue(const double Value)", m_sName));
		return false;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief sets the value with a data frame from a message
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CData::setValue(const std::vector<BYTE>& DataFrame)
	{
///@@@@@@@@@@@
///	printf("setValue(const std::vector<BYTE>& DataFrame<%04x>)\n", DataFrame[0]);

		if (!m_bInitialized)
			return false;

		long Value = 0;
		bool Negativ = false;

		if ((DataFrame[0] & 0x80) == 0x80)
			Negativ = true;
		else
			Negativ = false;

		switch (m_DataType)
		{

			case eDataTypeUnsignedChar:
				if (DataFrame.size() != 1)
				{
					VMC_Errors.push_back(CError("invalid data length", "CData::setValue(const std::vector<BYTE>& DataFrame)", m_sName));
					return false;
				}

				Value = DataFrame[0];
				break;

			case eDataTypeSignedChar:
				if (DataFrame.size() != 1)
				{
					VMC_Errors.push_back(CError("invalid data length", "CData::setValue(const std::vector<BYTE>& DataFrame)", m_sName));
					return false;
				}
				Value = DataFrame[0];
				if (Negativ)
					Value = Value - 0x100;
				break;

			case eDataTypeUnsignedInteger:
				if (DataFrame.size() != 2)
				{
					VMC_Errors.push_back(CError("invalid data length", "CData::setValue(const std::vector<BYTE>& DataFrame)", m_sName));
					return false;
				}
				Value = DataFrame[0];
				Value = (Value << 8) + DataFrame[1];
				break;

			case eDataTypeSignedInteger:
				if (DataFrame.size() != 2)
				{
					VMC_Errors.push_back(CError("invalid data length", "CData::setValue(const std::vector<BYTE>& DataFrame)", m_sName));
					return false;
				}
				Value = DataFrame[0];
				Value = (Value << 8) + DataFrame[1];
				if (Negativ)
					Value = Value - 0x10000;
				break;

			case eDataTypeSignedLong:
				if (DataFrame.size() != 4)
				{
					VMC_Errors.push_back(CError("invalid data length", "CData::setValue(const std::vector<BYTE>& DataFrame)", m_sName));
					return false;
				}
				Value = DataFrame[0];
				Value = (Value << 8) + DataFrame[1];
				Value = (Value << 8) + DataFrame[2];
				Value = (Value << 8) + DataFrame[3];
				break;

			case eDataTypeString:
				std::stringstream ErrorString;

				for (unsigned int i = 0; i < DataFrame.size(); i++)
				{		// stors VMC error string to a proper string
					ErrorString << DataFrame[i];
				}
				m_sName = ErrorString.str();
				//	VMC_Errors.push_back( CError("string not implemented yet", "CData::setValue(const std::vector<BYTE>& DataFrame)", _cName) );
				return false;
				break;
		}

		return setValue(Value);
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief return the value
///  @returns value
//////////////////////////////////////////////////////////////////////////////
	double CData::getValue() const
	{
		if (!m_bInitialized)
			return 0;

		return m_dValue;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief return the datatype
///  @returns datatype
//////////////////////////////////////////////////////////////////////////////
	EnumDataType CData::getDataType() const
	{
		return m_DataType;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief updates the timestamp by the momentary time
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	void CData::updateTimestamp()
	{
		if (!m_bInitialized)
			return;

		m_Timestamp.update();
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief return the timestamp
///  @returns timestamp
//////////////////////////////////////////////////////////////////////////////
	long double CData::getTimestamp() const
	{
		if (!m_bInitialized)
			return 0;

		return m_Timestamp.getTime();
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief sets the allowed range for the value
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////	
	bool CData::setRange(const double Maximum, const double Minimum)
	{
		if (!m_bInitialized)
			return false;

		if (Maximum >= Minimum)
		{
			m_dMaximum = Maximum;
			m_dMinimum = Minimum;
			return true;
		}
		VMC_Errors.push_back(CError("Maximum in not bigger than Minimum", "CData::setRange()", m_sName));
		return false;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief returns the maximum of the range
///  @returns maximum value
//////////////////////////////////////////////////////////////////////////////
	double CData::getMaximum() const
	{
		if (!m_bInitialized)
			return 0;

		return m_dMaximum;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief returns the minimum of the range
///  @returns minimum value
//////////////////////////////////////////////////////////////////////////////
	double CData::getMinimum() const
	{
		if (!m_bInitialized)
			return 0;

		return m_dMinimum;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief returns the name of the data
///  @returns name
//////////////////////////////////////////////////////////////////////////////
	std::string CData::getName() const
	{
		if (!m_bInitialized)
			return 0;

		return m_sName;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief returns motor channel number
///  @returns motor channel
//////////////////////////////////////////////////////////////////////////////
	unsigned int CData::getMotorChannel() const
	{
		if (!m_bInitialized)
			return 0;

		return m_MotorChannel.get();
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief sets the unit prefix for the value ( milli, kilo...)
//////////////////////////////////////////////////////////////////////////////
	void CData::setUnitPrefix(const EnumUnitPrefix& UnitPrefix)
	{
		if (!m_bInitialized)
			return;

		m_UnitPrefix = UnitPrefix;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief returns the unit prefix
///  @returns unit prefix
//////////////////////////////////////////////////////////////////////////////
	std::string CData::getUnitPrefix() const
	{
		if (!m_bInitialized)
			return "";

		switch (m_UnitPrefix)
		{
			case eUnitPrefixGIGA:
				return "G";
				break;
			case eUnitPrefixMEGA:
				return "M";
				break;
			case eUnitPrefixKILO:
				return "k";
				break;
			case eUnitPrefixHECTO:
				return "h";
				break;
			case eUnitPrefixDECA:
				return "da";
				break;
			case eUnitPrefixONE:
				return "";
				break;
			case eUnitPrefixDECI:
				return "d";
				break;
			case eUnitPrefixCENTI:
				return "c";
				break;
			case eUnitPrefixMILLI:
				return "m";
				break;
			case eUnitPrefixMICRO:
				return "µ";
				break;
			case eUnitPrefixNANO:
				return "n";
				break;
			default:
				return "";
				break;
		}
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief sets the unit for the value ( meter, volt...)
//////////////////////////////////////////////////////////////////////////////
	void CData::setUnit(const EnumPhysicalUnit& Unit)
	{
		if (!m_bInitialized)
			return;

		m_Unit = Unit;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief returns the unit
///  @returns unit prefix
//////////////////////////////////////////////////////////////////////////////
	std::string CData::getUnit() const
	{
		if (!m_bInitialized)
			return "";

		switch (m_Unit)
		{
			case ePhysicalUnitNONE:
				return "";
				break;
			case ePhysicalUnitVOLT:
				return "V";
				break;
			case ePhysicalUnitAMPERE:
				return "A";
				break;
			case ePhysicalUnitWATT:
				return "W";
				break;
			case ePhysicalUnitOHM:
				return "Ohm";
				break;
			case ePhysicalUnitRPM:
				return "RPM";
				break;
			case ePhysicalUnitSECOND:
				return "s";
				break;
			case ePhysicalUnitMETER:
				return "m";
				break;
			case ePhysicalUnitNEWTONMETER:
				return "Nm";
				break;
			case ePhysicalUnitDEGREECELSIUS:
				return "°C";
				break;
			case ePhysicalUnitNEWTONMETER_AMPERE:
				return "N/A";
				break;
			case ePhysicalUnitRPM_VOLT:
				return "RPM/V";
				break;
			case ePhysicalUnitRPM_NEWTONMETER:
				return "RPM/Nm";
				break;
			case ePhysicalUnitKELVINWATT:
				return "W/K";
				break;
			default:
				return "";
				break;

		}
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief updates the value in the API by reading from the VMC
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CData::Update() const
	{

		if (!m_bInitialized)
		{
			VMC_Errors.push_back(CError("not initialized", "CData::Update()", m_sName));
			return false;
		}
//@@@@@@@@@@@@@@@@@@@@@
//	printf ("cdata.Update.cpp - GR %x Cmd %x \r\n", m_CommandGroup, m_Command );

		CMessage MessageToSend(m_CommandGroup, m_Command);

		if (!MessageToSend.appendToDataFrame(m_MotorChannel.getForVMC(), eDataTypeUnsignedChar))
		{
			VMC_Errors.push_back(CError("out of datatype range", "CData::Update()", m_sName));
			return false;
		}

		///////////////////////////////////DEBUG///////////////////////////////////////////
#ifdef VMC_DEBUG
		std::cout <<" update " << MessageToSend << std::endl;			//debug only
#endif
		///////////////////////////////////DEBUG///////////////////////////////////////////

		if (false == m_pTrans->sendMessage(MessageToSend))
		{
			return false;
		}

		return true;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief sets a new value in the VMC but not here in the API
///  @returns true = OK / false = Error
/////////////////////////////////////////////////// ///////////////////////////
	bool CData::Set(const double Value) const
	{

		//@@@@@@@@@@@@@@@@@@

		if (!m_bInitialized)
		{
			VMC_Errors.push_back(CError("not initialized", "CData::Set()", m_sName));
			return false;
		}

		if (m_dMaximum < Value || m_dMinimum > Value)
		{
			VMC_Errors.push_back(CError("value out of range", "CData::Set()", m_sName));
			return false;
		}

		if (!m_MotorChannel.validChannel())
		{

			printf("valid channel %08x\n", m_MotorChannel.validChannel());

			VMC_Errors.push_back(CError("invalid motor channel", "CData::Set()", m_sName));
			return false;
		}

		CMessage MessageToSend(m_CommandGroup, m_Command);

		if (!MessageToSend.appendToDataFrame(m_MotorChannel.getForVMC(), eDataTypeUnsignedChar))
		{
			VMC_Errors.push_back(CError("out of datatype range", "CData::Set()", m_sName));
			return false;
		}

		if (!MessageToSend.appendToDataFrame(Value, m_DataType))
		{
			VMC_Errors.push_back(CError("out of datatype range", "CData::Set()", m_sName));
			return false;
		}

		if (m_CommandGroup == 0x52)
		{  // Motor Control Command
			if (!MessageToSend.appendToDataFrame(*m_pNextRequestCommand, eDataTypeUnsignedChar))
			{
				VMC_Errors.push_back(CError("out of datatype range", "CData::Set()", m_sName));
				return false;
			}
		}
//@@@@@@@@@@@@@@@@@q
		if (m_CommandGroup == 0x50)
		{  // Motor Control Command

//	printf ("cdata.group50.cpp - GR %x Cmd %x Value %x\r\n", m_CommandGroup, m_Command, Value );

			if (!MessageToSend.appendToDataFrame(*m_pNextRequestCommand, eDataTypeUnsignedChar))
			{
				VMC_Errors.push_back(CError("out of datatype range", "CData::Set()", m_sName));
				return false;
			}
		}

		///////////////////////////////////DEBUG///////////////////////////////////////////
#ifdef VMC_DEBUG
		std::cout <<" send " << MessageToSend << "\n";				//debug only
#endif
		///////////////////////////////////DEBUG///////////////////////////////////////////

		if (false == m_pTrans->sendMessage(MessageToSend))
		{
			return false;
		}

		return true;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief puts the name, motorchannel and value in a std::ostream
///  @returns std::ostream
//////////////////////////////////////////////////////////////////////////////
	std::ostream& operator<<(std::ostream& out, const CData& d)
	{

		out << std::dec << d.getName() << "[" << d.getMotorChannel() << "]" << ": ";
		out << d.getValue();
//	if(d.getUnitPrefix() != one)
//		out << d.getUnitPrefix();
//	if(d.getUnit() != none)
//		out << " " << d.getUnit();
		return out;
	}

}  // namespace VMC

//@}
