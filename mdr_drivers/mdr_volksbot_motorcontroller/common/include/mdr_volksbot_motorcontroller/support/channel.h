//////////////////////////////////////////////////////////////////////////////
///  @file CChannel.h
///  @class VMC::CChannel
///  @brief In this class the channel numbers of the motors are handled.
///  @note The channel of an object of this class have to be set, otherwise it is not valid.  
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////

#ifndef _CChannel_H_
#define _CChannel_H_

#include "enums.h"
#include <sstream>

namespace VMC
{

	class CChannel
	{
 	public:
		CChannel();
		CChannel(const unsigned int Channel);
		~CChannel()
		{
		}
		;

		bool set(const unsigned int Channel);
		unsigned int get() const;
		unsigned int getForVMC() const;
		unsigned int getMaxChannels() const
		{
			return sm_nChannelNo;
		}
		CChannel operator=(const CChannel&);
		CChannel operator=(const unsigned int&);
		bool operator==(const unsigned int&) const;
		CChannel operator+(const unsigned int);
		CChannel operator-(const unsigned int);
		CChannel operator++();
		CChannel operator--();
		bool validChannel() const;
		friend std::ostream& operator<<(std::ostream& out, const CChannel& ch);

 	private:
		bool m_bInitialized;
		const static unsigned int sm_nChannelNo;
		unsigned int m_nChannel;
	};

}  // namespace VMC
#endif //_CChannel_H_
