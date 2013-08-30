//////////////////////////////////////////////////////////////////////////////
///  @file CCommunicationLayer.cpp
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////

#include "layer/communication_layer.h"

namespace VMC
{

	using namespace fair;

//////////////////////////////////////////////////////////////////////////////
///  @brief standard constructor
//////////////////////////////////////////////////////////////////////////////
	CCommunicationLayer::CCommunicationLayer()
	{
		m_EDeviceState = closed;
		m_Adapter = NULL;
		m_EAdapterName = eAdapterNONE;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief standard destructor
//////////////////////////////////////////////////////////////////////////////
	CCommunicationLayer::~CCommunicationLayer()
	{
		deleteAdapter();
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief initilize the device so it�s ready to communicate
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CCommunicationLayer::initDevice()
	{

		if (strlen(m_cDeviceName) == 0)
		{
			VMC_Errors.push_back(CError("no device were selected", "COMMUNICATION_LAYER::InitDevice()"));
			return false;
		}
		if (createAdapter() == false)
			return false;

		m_Adapter->setReceiveMode(eUnbuffered);

		(reinterpret_cast<CComAdapter*>(m_Adapter))->setBaudrate(eB57600);

		if (m_EDeviceState == closed)
		{
			m_Adapter->setDeviceName(m_cDeviceName);

			if (m_Adapter->openDevice() != 0)
			{
				m_EDeviceState = open;
			}
			else
			{
				VMC_Errors.push_back(CError("unable to open device", "COMMUNICATION_LAYER::InitDevice()"));
				return false;
			}
		}
		return true;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief close the device no communication is possible afterwords
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CCommunicationLayer::closeDevice()
	{

		if (m_EDeviceState == open)
		{
			if (m_Adapter->closeDevice() != 0)
			{
				m_EDeviceState = closed;
			}
			else
			{
				VMC_Errors.push_back(CError("unable to close device", "COMMUNICATION_LAYER::CloseDevice()"));
				return false;
			}
		}
		return true;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief creates a new adapter like RS232, CAN...
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CCommunicationLayer::createAdapter()
	{

		if (strlen(m_cDeviceName) == 0)
			return false;

		if (m_Adapter == NULL)
		{
			switch (m_EAdapterName)
			{
				case eAdapterRS232:
					m_Adapter = new CComAdapter(m_cDeviceName);
					break;

				case eAdapterFILE:
					VMC_Errors.push_back(CError("FILE adapter is not implemented yet", "COMMUNICATION_LAYER::createAdapter()"));
					return false;
					break;

				case eAdapterCAN:
					VMC_Errors.push_back(CError("CAN adapter is not implemented yet", "COMMUNICATION_LAYER::createAdapter()"));
					return false;
					break;

				case eAdapterUSB:
					VMC_Errors.push_back(CError("USB adapter is not implemented yet", "COMMUNICATION_LAYER::createAdapter()"));
					return false;
					break;
				case eAdapterNONE:
					VMC_Errors.push_back(CError("NONE adapter was selected", "COMMUNICATION_LAYER::createAdapter()"));
					return false;
					break;
			}
		}
		return true;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief deletes the actual adapter
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CCommunicationLayer::deleteAdapter()
	{

		if (closeDevice() == false)
			return false;
		delete m_Adapter;
		m_Adapter = NULL;
		return true;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief sends data over the adapter
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CCommunicationLayer::send(std::string& Data)
	{

		if (initDevice() == false)
			return false;

		unsigned long sendByte = 0;

		sendByte = m_Adapter->send(const_cast<char*>(Data.c_str()), Data.length());
		if (sendByte != Data.length())
		{
			VMC_Errors.push_back(CError("not all bytes could be send", "COMMUNICATION_LAYER::send()"));
			return false;
		}
		return true;

	}

//////////////////////////////////////////////////////////////////////////////
///  @brief receives data over the adapter
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CCommunicationLayer::receive(std::string& receiveString)
	{

		if (initDevice() == false)
			return false;

		unsigned long receivedBytes;
		char receiveArray[RECEIVESIZE];

		receivedBytes = m_Adapter->receive(receiveArray, RECEIVESIZE);

		receiveString.append(receiveArray, receivedBytes);
		return true;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief selects a adapter (RS232, CAN...)
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CCommunicationLayer::selectAdapter(const EnumAdapter& Adapter)
	{

		if (deleteAdapter() == false)
			return false;

		m_EAdapterName = Adapter;

		if (createAdapter() == false)
			return false;

		return true;
	}

//////////////////////////////////////////////////////////////////////////////
///  @brief selects a device (COM1, COM2...)
///  @returns true = OK / false = Error
//////////////////////////////////////////////////////////////////////////////
	bool CCommunicationLayer::selectDevice(const char* DeviceName)
	{

		if (closeDevice() == false)
			return false;

		if ((strlen(DeviceName) + 1) >= MAX_DEVICE_NAME_LENGTH)
		{ /* +1 for null-terminator */
			VMC_Errors.push_back(CError("Device Name is to long", "COMMUNICATION_LAYER::selectDevice(const char* DeviceName)"));
			return false;
		}

		strcpy(m_cDeviceName, DeviceName);

		if (initDevice() == false)
			return false;

		return true;
	}

}  // namespace VMC

