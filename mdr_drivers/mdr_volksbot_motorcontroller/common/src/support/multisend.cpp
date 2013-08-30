//////////////////////////////////////////////////////////////////////////////
///  @file CMultisend.cpp
///  @author Jan Paulus
///  @version 0.1
///  @date 16.10.2006
//////////////////////////////////////////////////////////////////////////////

#include "support/multisend.h"

namespace VMC
{

//////////////////////////////////////////////////////////////////////////////
///  @brief Initialized the CData Object
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CMultisend::Init(CTranslationLayer* Trans, BYTE* pNextRequestCommand, const BYTE nCommandGroup, const BYTE nCommand, const EnumDataType DataType)
	{

		if (m_bInitialized)
			return false;

		m_pTrans = Trans;
		m_pNextRequestCommand = pNextRequestCommand;
		m_CommandGroup = nCommandGroup;
		m_Command = nCommand;
		m_DataType = DataType;

		m_bInitialized = true;
		return true;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief sets a new value in the VMC but not here in the API
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CMultisend::Set(const double Motor1, const double Motor2, const double Motor3, const double Motor4, const double Motor5, const double Motor6,
	                     const double Motor7, const double Motor8, const double Motor9, const double Motor10)
	{

		if (!m_bInitialized)
		{
			VMC_Errors.push_back(CError("not initialized", "MULTIPLESEND::Set()"));
			return false;
		}

		double Value;
		CChannel ch;
		CMessage MessageToSend(m_CommandGroup, m_Command);

		for (unsigned int i = 1; i <= ch.getMaxChannels(); i++)
		{

			switch (i)
			{
				case 1:
					Value = Motor1;
					break;
				case 2:
					Value = Motor2;
					break;
				case 3:
					Value = Motor3;
					break;
				case 4:
					Value = Motor4;
					break;
				case 5:
					Value = Motor5;
					break;
				case 6:
					Value = Motor6;
					break;
				case 7:
					Value = Motor7;
					break;
				case 8:
					Value = Motor8;
					break;
				case 9:
					Value = Motor9;
					break;
				case 10:
					Value = Motor10;
					break;
			}

			if (!MessageToSend.appendToDataFrame(Value, m_DataType))
			{
				VMC_Errors.push_back(CError("out of datatype range", "MULTIPLESEND::Set()"));
				return false;
			}
		}

		if (m_CommandGroup == 0x52)
		{  // Motor Control Command

			if (!MessageToSend.appendToDataFrame(*m_pNextRequestCommand, eDataTypeUnsignedChar))
			{
				VMC_Errors.push_back(CError("out of datatype range", "MULTIPLESEND::Set()"));
				return false;
			}
		}

		///////////////////////////////////DEBUG///////////////////////////////////////////
#ifdef VMC_DEBUG
		std::cout <<"multisend " << MessageToSend << "\n";				//debug only
#endif
		///////////////////////////////////DEBUG///////////////////////////////////////////

		if (false == m_pTrans->sendMessage(MessageToSend))
		{
			VMC_Errors.push_back(CError("unable to send to adapter", "MULTIPLESEND::Set()"));
			return false;
		}

		return true;
	}

}  // namespace VMC

//@}
