//////////////////////////////////////////////////////////////////////////////
///  @file CMultisend.h
///  @class VMC::CMultisend
///	 @brief With an object of this class it is possible to set multiple motor with one message.
///  @author Jan Paulus
///  @version 0.1
///  @date 16.10.2006
//////////////////////////////////////////////////////////////////////////////

#ifndef _CMultisend_H_
#define _CMultisend_H_

#include "message.h"
#include "channel.h"
#include "enums.h"
#include "../layer/translation_layer.h"
#include "error.h"
#include <string>
#include <list>

namespace VMC
{

	extern std::list<CError> VMC_Errors;

	class CMultisend
	{
 	public:

		CMultisend()
		{
			m_bInitialized = false;
		}

		CMultisend(CTranslationLayer* Trans, BYTE* pNextRequestCommand, const BYTE nCommandGroup, const BYTE nCommand, const EnumDataType DataType)
				: m_pTrans(Trans),
				  m_pNextRequestCommand(pNextRequestCommand),
				  m_CommandGroup(nCommandGroup),
				  m_Command(nCommand),
				  m_DataType(DataType)
		{
			m_bInitialized = true;
		}

		bool Init(CTranslationLayer* Trans, BYTE* pNextRequestCommand, const BYTE nCommandGroup, const BYTE nCommand, const EnumDataType DataType);

		~CMultisend()
		{
		}
		;

		EnumDataType getDataType() const;

		bool Set(const double Motor1 = 0, const double Motor2 = 0, const double Motor3 = 0, const double Motor4 = 0, const double Motor5 = 0,
		         const double Motor6 = 0, const double Motor7 = 0, const double Motor8 = 0, const double Motor9 = 0, const double Motor10 = 0);

 	private:
		bool m_bInitialized;
		CTranslationLayer* m_pTrans;
		BYTE* m_pNextRequestCommand;
		BYTE m_CommandGroup;
		BYTE m_Command;
		EnumDataType m_DataType;
	};

}  // namespace VMC

#endif //_CMultisend_H_
//@}

