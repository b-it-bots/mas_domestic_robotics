//////////////////////////////////////////////////////////////////////////////
///  @file CCommunicationLayer.h
///  @class VMC::CCommunicationLayer
///  @brief handels the whole communication with the VMC
///  @note The adapters (RS232, USB...) will be seleced here and also the devices (COM1, COM2...)
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////

#ifndef _CCommunicationLayer_H_
#define _CCommunicationLayer_H_

#include <string>
#include <list>

#include "support/enums.h"
#include "support/error.h"

#include "interface_device_adapter.h"
#include "com_adapter.h"

namespace VMC
{

	extern std::list<CError> VMC_Errors;

#define RECEIVESIZE 1000

#define MAX_DEVICE_NAME_LENGTH 255

	class CCommunicationLayer
	{
 	public:
		CCommunicationLayer();

		~CCommunicationLayer();

		bool send(std::string& Data);

		bool receive(std::string& receiveString);

		bool selectAdapter(const EnumAdapter& Adapter);

		bool selectDevice(const char* DeviceName);

		bool initDevice();
		bool closeDevice();
 	private:

		fair::IDeviceAdapter* m_Adapter;

		char m_cDeviceName[MAX_DEVICE_NAME_LENGTH];

		EnumAdapter m_EAdapterName;

		enum EnumDeviceState
		{
			closed,
			open
		};

		EnumDeviceState m_EDeviceState;

		bool createAdapter();
		bool deleteAdapter();

	};

}  // namespace VMC
#endif //_CCommunicationLayer_H_
