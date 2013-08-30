//////////////////////////////////////////////////////////////////////////////
///  @file CMessage.h
///  @class VMC::CMessage
///  @brief An object of this class represent a VMC message.
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////

#ifndef _CMessage_H_
#define _CMessage_H_

#include "enums.h"
#include "timestamp.h"
#include <vector>
#include <iostream>

namespace VMC
{

	class CMessage
	{
 	public:
		CMessage();
		CMessage(const BYTE CommandGroup, const BYTE Command, const BYTE PacketCounter = 0)
				: m_CommandGroup(CommandGroup),
				  m_Command(Command),
				  m_PacketCounter(PacketCounter)
		{
			CalculateCRC();
		}

		CMessage(const BYTE CommandGroup, const BYTE Command, const std::vector<BYTE>& DataFrame, const BYTE PacketCounter = 0)
				: m_CommandGroup(CommandGroup),
				  m_Command(Command),
				  m_DataFrame(DataFrame),
				  m_PacketCounter(PacketCounter)
		{
			CalculateCRC();
		}

		~CMessage()
		{
		}
		;

		void setPacketCounter(const BYTE PacketCounter);
		BYTE getPacketCounter() const;

		void setCommandGroup(const BYTE CommandGroup);
		BYTE getCommandGroup() const;

		BYTE getCommand() const;
		void setCommand(const BYTE Command);

		void getDataFrame(std::vector<BYTE>& DataFrame) const;
		void setDataFrame(const std::vector<BYTE>& DataFrame);

		bool appendToDataFrame(const double& value, const EnumDataType& DataType);

		void CalculateCRC();
		BYTE getCRC() const;
		void setCRC(const BYTE CRC);

		bool isResponseMessage() const;
		bool verifyCRC() const;

		bool isValidMessage() const;

		void clear();

		void printHex() const;
		friend std::ostream& operator<<(std::ostream& out, const CMessage& m);
 	private:

		BYTE m_CommandGroup;
		BYTE m_Command;
		std::vector<BYTE> m_DataFrame;

		BYTE m_PacketCounter;
		BYTE m_CRC;
	};

}  // namespace VMC
#endif //_CMessage_H_
