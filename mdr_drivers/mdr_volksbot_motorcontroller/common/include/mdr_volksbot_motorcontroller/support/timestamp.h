//////////////////////////////////////////////////////////////////////////////
///  @file CTimestamp.h
///  @class VMC::CTimestamp
///  @brief An object of this class represent an timestamp.
///  @note This timestamp is the time in milli seconds, since the start of the program.
///  @author Jan Paulus
///  @version 0.2
///  @date 16.01.2007
//////////////////////////////////////////////////////////////////////////////

#ifndef _CTimestamp_H_
#define _CTimestamp_H_

#include <iostream>

#include "timer.h"

namespace VMC
{

	class CTimestamp
	{
 	public:

		CTimestamp()
		{
			m_dMilliTime = sm_globalMilliTime.getTime();
		}

		~CTimestamp()
		{
		}
		;

		long double getTime() const
		{
			return m_dMilliTime;
		}

		void printTime() const;

		void update()
		{
			m_dMilliTime = sm_globalMilliTime.getTime();
		}

		friend std::ostream& operator<<(std::ostream& out, const CTimestamp& t);

 	private:

		static fair::CTimer sm_globalMilliTime;

		long double m_dMilliTime;
	};

}  // namespace VMC
#endif //_CTimestamp_H_
