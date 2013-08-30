//////////////////////////////////////////////////////////////////////////////
///  @file CTranslationLayer.cpp
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////

#include "layer/translation_layer.h"

namespace VMC
{

	const BYTE CTranslationLayer::sm_PStart = 0x7B;
	const BYTE CTranslationLayer::sm_PEnd = 0x7D;
	const BYTE CTranslationLayer::sm_Quote = 0x5C;

	CTranslationLayer::CTranslationLayer()
	{
		m_SendPacketCounter = 0;
		m_ReceivePacketCounter = 0;
		m_bReceivedMessage = false;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief appends a byte to the string
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	void CTranslationLayer::appendByte(std::string& MessageString, BYTE byte)
	{

		if (byte == sm_PStart || byte == sm_PEnd || byte == sm_Quote)
		{
			MessageString.append(1, sm_Quote);
			MessageString.append(1, byte);
		}
		else
		{
			MessageString.append(1, byte);
		}
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief sends a CMessage object to the VMC
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CTranslationLayer::sendMessage(const CMessage& Message)
	{

		std::string MessageString;
		std::vector<BYTE> DataFrame;
		unsigned int i;

		/*
		 ///////////////////////////////////DEBUG///////////////////////////////////////////
		 #ifdef VMC_DEBUG
		 CMessage Messagecopy;
		 Messagecopy = Message;
		 Messagecopy.setPacketCounter(_SendPacketCounter);
		 std::cout <<"     send " << Messagecopy << "\n";				//debug only
		 #endif
		 ///////////////////////////////////DEBUG///////////////////////////////////////////
		 */
		MessageString = sm_PStart;

		appendByte(MessageString, m_SendPacketCounter);
		appendByte(MessageString, Message.getCommandGroup());
		appendByte(MessageString, Message.getCommand());
//@@@@@@@@@
		Message.getDataFrame(DataFrame);

//printf("Size<%08x> first<%08x>\n", DataFrame.size(), DataFrame[0]);  

		Message.getDataFrame(DataFrame);
		for (i = 0; i < DataFrame.size(); i++)
		{

			appendByte(MessageString, DataFrame[i]);
		}

		appendByte(MessageString, Message.getCRC());

		MessageString.append(1, sm_PEnd);
		//std::cout << "send: "<< MessageString << "\n";

		if (m_SendPacketCounter == 254)
			m_SendPacketCounter = 0;
		else
			m_SendPacketCounter = m_SendPacketCounter + 1;

		return m_Com.send(MessageString);
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief receives data from the VMC and stores is in the ReceiveBuffer
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CTranslationLayer::receiveMessage()
	{
		return m_Com.receive(m_sReceiveBuffer);
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief Open Device
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CTranslationLayer::openDevice()
	{
		return m_Com.initDevice();
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief Close Device
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CTranslationLayer::closeDevice()
	{
		return m_Com.closeDevice();
	}
//////////////////////////////////////////////////////////////////////////////
///  @brief finds a message in the ReceiveBuffer
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CTranslationLayer::findMessage(CMessage& NewMessage)
	{
		int i;
		int IndexStart = -1;
		int IndexEnd = -1;
		std::vector<BYTE> DataFrame;

		NewMessage.clear();

		//std::cout << "bevor: " <<_ReceiveBuffer.size() << "\n";
		// find a whole message
		for (i = 0; (unsigned int) i < m_sReceiveBuffer.size(); i++)
		{
			if (m_sReceiveBuffer[i] == sm_Quote)
			{
				if ((unsigned int) i + 2 >= m_sReceiveBuffer.size())
					break;
				i = i + 2;						//skip next byte
			}
			if (m_sReceiveBuffer[i] == sm_PStart)
			{
				IndexStart = i;
			}
			if (m_sReceiveBuffer[i] == sm_PEnd && IndexStart != -1)
			{
				IndexEnd = i;
				break;
			}
		}

		if (IndexStart == -1 || IndexEnd == -1)
		{		// return if no Message is found
			if (m_sReceiveBuffer.size() > 5000)
				VMC_Errors.push_back(CError("Receive Buffer bigger than 5000 byte", "TRANSLATION_LAYER::findMessage()"));
			return false;
		}

		// Erease all Quote in the string
		for (i = IndexStart; i <= IndexEnd; i++)
		{
			if (m_sReceiveBuffer[i] == sm_Quote)
			{
				m_sReceiveBuffer.erase(m_sReceiveBuffer.begin() + i);	//delete quote
				IndexEnd = IndexEnd - 1;							// End marker have to deduct by one
				i++;
			}
		}

		NewMessage.setPacketCounter(m_sReceiveBuffer[IndexStart + 1]);
		NewMessage.setCommandGroup(m_sReceiveBuffer[IndexStart + 2]);
		NewMessage.setCommand(m_sReceiveBuffer[IndexStart + 3]);

		if (IndexStart + 4 < IndexEnd - 2)
		{

			for (i = (IndexStart + 4); i <= (IndexEnd - 2); i++)
			{
				DataFrame.push_back(m_sReceiveBuffer[i]);
			}
			NewMessage.setDataFrame(DataFrame);
		}

		NewMessage.setCRC(m_sReceiveBuffer[IndexEnd - 1]);

		//delete read message out of the _ReceiveBuffer
		m_sReceiveBuffer.erase(m_sReceiveBuffer.begin(), m_sReceiveBuffer.begin() + (IndexEnd + 1));

		// increasing packet counter by one, with a overflow at 255
		if (m_ReceivePacketCounter == 255)
			m_ReceivePacketCounter = 0;
		else
			m_ReceivePacketCounter = m_ReceivePacketCounter + 1;

		// checking if packet counter is the same or if a message was missed
		if (m_ReceivePacketCounter != NewMessage.getPacketCounter())
		{
			m_ReceivePacketCounter = NewMessage.getPacketCounter();
			if (m_bReceivedMessage)
				VMC_Errors.push_back(CError("packet counter mismatch probably missed a message", "TRANSLATION_LAYER::findMessage()"));
		}
		m_bReceivedMessage = true;

		return true;
	}

}  // namespace VMC
