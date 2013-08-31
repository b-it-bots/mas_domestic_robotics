//////////////////////////////////////////////////////////////////////////////
///  @file CChannel.cpp
///  @author Jan Paulus
///  @version 0.1
///  @date 16.10.2006
//////////////////////////////////////////////////////////////////////////////

#include "support/channel.h"

namespace VMC
{

	const unsigned int CChannel::sm_nChannelNo = 3;

//////////////////////////////////////////////////////////////////////////////
///  @brief standard constructor
//////////////////////////////////////////////////////////////////////////////
	CChannel::CChannel()
	{
		m_nChannel = 0;
		m_bInitialized = false;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief constructor
//////////////////////////////////////////////////////////////////////////////
	CChannel::CChannel(const unsigned int Channel)
	{
		if ((Channel < sm_nChannelNo) && (Channel >= 0))
		{
			m_nChannel = Channel;
			m_bInitialized = true;
		}
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief sets the channel number
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CChannel::set(const unsigned int Channel)
	{
		if ((Channel < sm_nChannelNo) && (Channel >= 0))
		{
			m_nChannel = Channel;
			m_bInitialized = true;
			return true;
		}
		return false;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief returns the channel number
//////////////////////////////////////////////////////////////////////////////
	unsigned int CChannel::getForVMC() const
	{
		return m_nChannel + 1;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief returns the channel number
//////////////////////////////////////////////////////////////////////////////
	unsigned int CChannel::get() const
	{
		return m_nChannel;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief equal operator
///  @returns CChannel object
//////////////////////////////////////////////////////////////////////////////
	CChannel CChannel::operator=(const CChannel& ch)
	{
		m_nChannel = ch.get();
		m_bInitialized = true;
		return *this;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief equal operator
///  @returns CChannel object
//////////////////////////////////////////////////////////////////////////////
	CChannel CChannel::operator=(const unsigned int& i)
	{
		if ((i < sm_nChannelNo) && (i >= 0))
		{
			m_nChannel = i;
			m_bInitialized = true;
		}
		return *this;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief compare operator
///  @returns true = equal / false = not equal
//////////////////////////////////////////////////////////////////////////////
	bool CChannel::operator==(const unsigned int& i) const
	{
		if (i == m_nChannel)
			return true;

		return false;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief plus operator
///  @returns CChannel object
//////////////////////////////////////////////////////////////////////////////
	CChannel CChannel::operator+(const unsigned int i)
	{
		if (((m_nChannel + i) < sm_nChannelNo) && ((m_nChannel + i) >= 0))
		{
			m_nChannel = m_nChannel + i;
			m_bInitialized = true;
		}

		return *this;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief minus operator
///  @returns CChannel object
//////////////////////////////////////////////////////////////////////////////
	CChannel CChannel::operator-(const unsigned int i)
	{
		if (((m_nChannel - i) < sm_nChannelNo) && ((m_nChannel - i) >= 0))
		{
			m_nChannel = m_nChannel - i;
			m_bInitialized = true;
		}

		return *this;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief adds one to the channel number
///  @returns CChannel object
//////////////////////////////////////////////////////////////////////////////
	CChannel CChannel::operator++()
	{
		if (((m_nChannel + 1) < sm_nChannelNo) && ((m_nChannel + 1) >= 0))
		{
			m_nChannel = m_nChannel + 1;
			m_bInitialized = true;
		}

		return *this;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief subtracts one from the channel number
///  @returns CChannel object
//////////////////////////////////////////////////////////////////////////////
	CChannel CChannel::operator--()
	{
		if (((m_nChannel - 1) < sm_nChannelNo) && ((m_nChannel - 1) >= 0))
		{
			m_nChannel = m_nChannel - 1;
			m_bInitialized = true;
		}

		return *this;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief validates if a CChannel object is OK
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CChannel::validChannel() const
	{

//	printf("validChannel--()<%06d><%06d><%06d>\n", m_nChannel, sm_nChannelNo, m_bInitialized);

		if ((m_nChannel < sm_nChannelNo) && (m_nChannel >= 0) && (m_bInitialized == true))
			return true;
//	printf("FALSE-------validChannel--()<%06d><%06d><%06d>\n", m_nChannel, sm_nChannelNo, m_bInitialized);

		return false;
	}

	std::ostream& operator<<(std::ostream& out, const CChannel& ch)
	{

		out << ch.get();
		return out;
	}

}  // namespace VMC
