#include "support/timestamp.h"

namespace VMC
{

	fair::CTimer CTimestamp::sm_globalMilliTime;

	void CTimestamp::printTime() const
	{

		std::cout << std::dec;
		std::cout << m_dMilliTime << "ms";
	}

	std::ostream& operator<<(std::ostream& out, const CTimestamp& t)
	{
		out << std::dec;
		out << t.getTime();
		return out;
	}

}  // namespace VMC
