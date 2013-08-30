//////////////////////////////////////////////////////////////////////////////
///  @file CSendTwo.h
///  @class VMC::CSendTwo
///  @brief Sends two Parameters at a time (example: linear and angular velocity)
///  @author Jan Paulus
///  @version 0.1
///  @date 16.10.2006
//////////////////////////////////////////////////////////////////////////////

#ifndef _CSendTwo_H_
#define _CSendTwo_H_

#include "message.h"
#include "enums.h"
#include "../layer/translation_layer.h"
#include "error.h"
#include <string>
#include <list>

namespace VMC
{

	extern std::list<CError> VMC_Errors;

	class CSendTwo
	{
 	public:

		CSendTwo()
		{
			m_bInitialized = false;
		}

		CSendTwo(CTranslationLayer* Trans, BYTE* pNextRequestCommand, const BYTE nCommandGroup, const BYTE nCommand, const EnumDataType DataType)
				: m_pTrans(Trans),
				  m_pNextRequestCommand(pNextRequestCommand),
				  m_CommandGroup(nCommandGroup),
				  m_Command(nCommand),
				  m_DataType(DataType)
		{
			m_bInitialized = true;
		}

		bool Init(CTranslationLayer* Trans, BYTE* pNextRequestCommand, const BYTE nCommandGroup, const BYTE nCommand, const EnumDataType DataType);

		~CSendTwo()
		{
		}
		;

		EnumDataType getDataType() const;

		bool Set(const double Value1, const double Value2);

 	private:
		bool m_bInitialized;
		CTranslationLayer* m_pTrans;
		BYTE* m_pNextRequestCommand;
		BYTE m_CommandGroup;
		BYTE m_Command;
		EnumDataType m_DataType;
	};

}  // namespace VMC

#endif //_CSendTwo_H_
//@}

