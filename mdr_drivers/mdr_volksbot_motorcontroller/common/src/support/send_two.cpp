//////////////////////////////////////////////////////////////////////////////
///  @file CSendTwo.cpp
///  @author Jan Paulus
///  @version 0.1
///  @date 16.10.2006
//////////////////////////////////////////////////////////////////////////////

#include "support/send_two.h"

namespace VMC
{

//////////////////////////////////////////////////////////////////////////////
///  @brief Initialized the CData Object
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CSendTwo::Init(CTranslationLayer* Trans, BYTE* pNextRequestCommand, const BYTE nCommandGroup, const BYTE nCommand, const EnumDataType DataType)
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
	bool CSendTwo::Set(const double Value1, const double Value2)
	{

		if (!m_bInitialized)
		{
			VMC_Errors.push_back(CError("not initialized", "CStendTwo::Set()"));
			return false;
		}

		CMessage MessageToSend(m_CommandGroup, m_Command);

		if (!MessageToSend.appendToDataFrame(Value1, m_DataType))
		{
			VMC_Errors.push_back(CError("out of datatype range", "CStendTwo::Set()"));
			return false;
		}

		if (!MessageToSend.appendToDataFrame(Value2, m_DataType))
		{
			VMC_Errors.push_back(CError("out of datatype range", "CStendTwo::Set()"));
			return false;
		}

		if (m_CommandGroup == 0x52)
		{  // Motor Control Command
			if (!MessageToSend.appendToDataFrame(*m_pNextRequestCommand, eDataTypeUnsignedChar))
			{
				VMC_Errors.push_back(CError("out of datatype range", "CStendTwo::Set()"));
				return false;
			}
		}

		///////////////////////////////////DEBUG///////////////////////////////////////////
#ifdef VMC_DEBUG
		std::cout <<"  sendtwo " << MessageToSend << "\n";				//debug only
#endif
		///////////////////////////////////DEBUG///////////////////////////////////////////

		if (false == m_pTrans->sendMessage(MessageToSend))
		{
			VMC_Errors.push_back(CError("unable to send to adapter", "CStendTwo::Set()"));
			return false;
		}

		return true;
	}

}  // namespace VMC

//@}
