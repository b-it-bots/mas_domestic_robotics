//////////////////////////////////////////////////////////////////////////////
///  @file CRequest.h
///  @class VMC::CRequest
///  @brief An object of this class represents a request to the VMC. It also includes the buildup of the corresponding response message.
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////

#ifndef _CRequest_H_
#define _CRequest_H_

#include <vector>
#include "channel.h"
#include "data.h"
#include "enums.h"

namespace VMC
{
	class CRequest
	{
 	public:
		CRequest();

		CRequest(BYTE CommandGroup, BYTE Command, std::vector<CData*>& ResponseBuildup, bool ActiveSwitch = false)
				: m_CommandGroup(CommandGroup),
				  m_Command(Command),
				  m_ResponseBuildup(ResponseBuildup),
				  m_bAktive(ActiveSwitch)
		{
		}
		;

		~CRequest()
		{
		}
		;

		BYTE getCommandGroup() const
		{
			return m_CommandGroup;
		}
		BYTE getCommand() const
		{
			return m_Command;
		}

		void setActiveSwitch(const bool SwitchState);
		bool getActiveSwitchState() const;

		CData* getResponseBuildup(const unsigned int DataNo) const;
		size_t getResponseBuildupSize() const
		{
			return m_ResponseBuildup.size();
		}

		friend std::ostream& operator<<(std::ostream& out, const CRequest& e);

 	private:

		BYTE m_CommandGroup;
		BYTE m_Command;

		std::vector<CData*> m_ResponseBuildup;
		bool m_bAktive;

	};

}  // namespace VMC
#endif //_CRequest_H_
