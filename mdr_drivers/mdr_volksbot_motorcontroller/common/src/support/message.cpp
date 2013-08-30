//////////////////////////////////////////////////////////////////////////////
///  @file CMessage.cpp
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////

#include "support/message.h"

namespace VMC
{

//////////////////////////////////////////////////////////////////////////////
///  @brief standard constructor
//////////////////////////////////////////////////////////////////////////////
	CMessage::CMessage()
	{
		m_CommandGroup = 0;
		m_Command = 0;
		m_PacketCounter = 0;
		m_CRC = 0;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief sets the packet counter for this message
//////////////////////////////////////////////////////////////////////////////
	void CMessage::setPacketCounter(const BYTE PacketCounter)
	{
		m_PacketCounter = PacketCounter;
		CalculateCRC();
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief return the packet counter
///  @returns packet counter
//////////////////////////////////////////////////////////////////////////////
	BYTE CMessage::getPacketCounter() const
	{
		return m_PacketCounter;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief sets the command group
//////////////////////////////////////////////////////////////////////////////
	void CMessage::setCommandGroup(const BYTE CommandGroup)
	{
		m_CommandGroup = CommandGroup;
		CalculateCRC();
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief returns the command group
///  @returns command group
//////////////////////////////////////////////////////////////////////////////
	BYTE CMessage::getCommandGroup() const
	{
		return m_CommandGroup;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief returns command number
///  @returns command
//////////////////////////////////////////////////////////////////////////////
	BYTE CMessage::getCommand() const
	{
		return m_Command;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief sets the command number
//////////////////////////////////////////////////////////////////////////////	
	void CMessage::setCommand(const BYTE Command)
	{
		m_Command = Command;
		CalculateCRC();
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief returns the data frame of the message
//////////////////////////////////////////////////////////////////////////////
	void CMessage::getDataFrame(std::vector<BYTE>& DataFrame) const
	{
		DataFrame = m_DataFrame;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief sets the data frame
//////////////////////////////////////////////////////////////////////////////
	void CMessage::setDataFrame(const std::vector<BYTE>& DataFrame)
	{
		m_DataFrame = DataFrame;
		CalculateCRC();
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief calculates the CRC
//////////////////////////////////////////////////////////////////////////////
	void CMessage::CalculateCRC()
	{
		unsigned long sum = 0;
		for (unsigned int i = 0; i < m_DataFrame.size(); i++)
		{
			sum = sum + m_DataFrame[i];
		}
		sum = sum + m_CommandGroup + m_Command;  //+ _nPacketCounter;
		m_CRC = static_cast<BYTE>(sum % 255);			// can also be 256
		//the one byte checksum of this array can be calculated by adding all values,
		//than dividing it by 256 and keeping the remainder.
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief returns the CRC value
///  @returns CRC
//////////////////////////////////////////////////////////////////////////////
	BYTE CMessage::getCRC() const
	{
		return m_CRC;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief sets the CRC value
//////////////////////////////////////////////////////////////////////////////
	void CMessage::setCRC(const BYTE CRC)
	{
		m_CRC = CRC;
	}
//////////////////////////////////////////////////////////////////////////////
///  @brief sets message to zero
//////////////////////////////////////////////////////////////////////////////
	void CMessage::clear()
	{
		m_DataFrame.clear();
		m_CommandGroup = 0;
		m_Command = 0;
		m_PacketCounter = 0;
		m_CRC = 0;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief append a number
//////////////////////////////////////////////////////////////////////////////
	bool CMessage::appendToDataFrame(const double& Value, const EnumDataType& DataType)
	{

		BYTE Helper1;
		int Helper2;
		long Helper3;
		bool Negativ = false;

		if (Value < 0)
			Negativ = true;
		else
			Negativ = false;

		switch (DataType)
		{

			case eDataTypeUnsignedChar:
				if (Value < 0 || Value > 255)
					return false;

				m_DataFrame.push_back(static_cast<BYTE>(Value + 0.5));
				break;

			case eDataTypeSignedChar:
				if (Value < -127 || Value > 127)
					return false;

				if (Negativ)
					Helper1 = static_cast<BYTE>(Value - 0.5);
				else
					Helper1 = static_cast<BYTE>(Value + 0.5);
				m_DataFrame.push_back(Helper1);
				break;

			case eDataTypeUnsignedInteger:
				if (Value < 0 || Value > 65535)
					return false;

				Helper2 = static_cast<int>(Value + 0.5);
				m_DataFrame.push_back(static_cast<BYTE>(Helper2 >> 8));
				m_DataFrame.push_back(static_cast<BYTE>(Helper2));
				break;

			case eDataTypeSignedInteger:
				if (Value < -32767 || Value > 32767)
					return false;

				if (Negativ)
					Helper2 = static_cast<int>(Value - 0.5);
				else
					Helper2 = static_cast<int>(Value + 0.5);
				m_DataFrame.push_back(static_cast<BYTE>(Helper2 >> 8));
				m_DataFrame.push_back(static_cast<BYTE>(Helper2));
				break;

			case eDataTypeSignedLong:
				if (Value < -2147483647 || Value > 2147483647)
					return false;

				if (Negativ)
					Helper3 = static_cast<long>(Value - 0.5);
				else
					Helper3 = static_cast<long>(Value + 0.5);
				m_DataFrame.push_back(static_cast<BYTE>(Helper3 >> 24));
				m_DataFrame.push_back(static_cast<BYTE>(Helper3 >> 16));
				m_DataFrame.push_back(static_cast<BYTE>(Helper3 >> 8));
				m_DataFrame.push_back(static_cast<BYTE>(Helper3));
				break;

			case eDataTypeString:
				return false;
				break;
		}

		return true;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief identifies if the message is a status response message or not
///  @returns true = status response message / false = no status response message
//////////////////////////////////////////////////////////////////////////////
	bool CMessage::isResponseMessage() const
	{
		if (m_CommandGroup == 0x55)
			return true;
		else
			return false;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief verify the CRC
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CMessage::verifyCRC() const
	{
		unsigned long sum = 0;

		for (unsigned int i = 0; i < m_DataFrame.size(); i++)
		{
			sum = sum + m_DataFrame[i];
		}
		sum = sum + m_CommandGroup + m_Command;  //+ _nPacketCounter;
		if (m_CRC == sum % 255)		// can also be 256
			return true;
		else
			return false;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief prints the message to std::cout
//////////////////////////////////////////////////////////////////////////////
	void CMessage::printHex() const
	{
		std::cout << std::showbase;
		std::cout << "P_START:0x7B ";
		std::cout << "COUNT:" << std::hex << (int) m_PacketCounter << " ";
		std::cout << "CMDGRP:" << std::hex << (int) m_CommandGroup << " ";
		std::cout << "CMD:" << std::hex << (int) m_Command << " ";

		std::cout << "DATA:";
		for (unsigned int i = 0; i < m_DataFrame.size(); i++)
		{
			std::cout << std::hex << (int) m_DataFrame[i] << " ";
		}
		std::cout << "CRC:" << std::hex << (int) m_CRC << " ";
		std::cout << "P_END:0x7D" << "\n";

	}

//////////////////////////////////////////////////////////////////////////////
///  @brief prints the message to std::ostream
//////////////////////////////////////////////////////////////////////////////
	std::ostream& operator<<(std::ostream& out, const CMessage& m)
	{

		out << std::showbase;
		out << "P_START:0x7B ";
		out << "COUNT:" << std::hex << (int) m.getPacketCounter() << " ";
		out << "CMDGRP:" << std::hex << (int) m.getCommandGroup() << " ";
		out << "CMD:" << std::hex << (int) m.getCommand() << " ";

		out << "DATA:";
		std::vector<BYTE> DataFrame;
		m.getDataFrame(DataFrame);
		for (unsigned int i = 0; i < DataFrame.size(); i++)
		{
			out << std::hex << (int) DataFrame[i] << " ";
		}
		out << "CRC:" << std::hex << (int) m.getCRC() << " ";
		out << "P_END:0x7D | ";
		CTimestamp timestamp;
		out << timestamp;

		return out;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief verifies if Message is valid
///  @returns true = valid / false = not valid
//////////////////////////////////////////////////////////////////////////////
	bool CMessage::isValidMessage() const
	{

		if (m_CommandGroup != 0 && m_Command != 0)
			return true;
		else
			return false;
	}

}  // namespace VMC
