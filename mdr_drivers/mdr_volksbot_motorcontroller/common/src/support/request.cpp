//////////////////////////////////////////////////////////////////////////////
///  @file CRequest.cpp
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////

#include "support/request.h"

namespace VMC
{

//////////////////////////////////////////////////////////////////////////////
///  @brief standard constructor
//////////////////////////////////////////////////////////////////////////////
	CRequest::CRequest()
	{
		m_CommandGroup = 0;
		m_Command = 0;
		m_bAktive = false;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief sets the request active of inactive
//////////////////////////////////////////////////////////////////////////////
	void CRequest::setActiveSwitch(const bool SwitchState)
	{
		m_bAktive = SwitchState;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief returns if the request is active or not
///  @returns true = active / false = inactive
//////////////////////////////////////////////////////////////////////////////
	bool CRequest::getActiveSwitchState() const
	{
		return m_bAktive;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief return the buildup of the request data
///  @returns pointer to CData object 
//////////////////////////////////////////////////////////////////////////////
	CData* CRequest::getResponseBuildup(const unsigned int DataNo) const
	{
		return m_ResponseBuildup[DataNo];
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief prints the request to std::ostream
///  @returns std::ostream
//////////////////////////////////////////////////////////////////////////////
	std::ostream& operator<<(std::ostream& out, const CRequest& e)
	{

		if (e.m_bAktive)
		{
			out << "ACTIVE";
		}
		else
		{
			out << "DISABLED";
		}

		out << std::showbase;

		out << " CMD_GRP:" << std::hex << (int) e.m_CommandGroup;
		out << " CMD:" << std::hex << (int) e.m_Command;

		int i = 0;

		while (i < e.m_ResponseBuildup.size())
		{

			out << " Data" << i << ":" << e.m_ResponseBuildup[i]->getName();

			out << "_" << std::dec << e.m_ResponseBuildup[i]->getMotorChannel() + 1;
			i++;
		}
		out << "\n";
		return out;

	}

}  // namespace VMC
