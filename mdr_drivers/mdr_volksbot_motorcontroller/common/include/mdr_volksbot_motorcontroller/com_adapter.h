#ifndef COMADAPTER_H__
#define COMADAPTER_H__

#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/signal.h>
#include <sys/types.h>

#include "interface_device_adapter.h"
#include "common.h"

#ifdef WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

#include <linux/serial.h>
#include <sys/select.h>
#endif

namespace fair
{

#ifdef WIN32
	/**
	 * Possible COM port baudrates
	 */
	enum EnumBaudrate
	{	eB9600 = 9600, eB19200 = 19200, eB38400 = 38400, eB57600 = 57600, eB115200 = 115200, eB500000 = 500000};

#define DEVHANDLE HANDLE
#define COMMSETTINGS DCB

#else
	/**
	 * Possible COM port baudrates
	 */
	enum EnumBaudrate
	{
		eB50 = 0000001,
		eB75 = 0000002,
		eB110 = 0000003,
		eB134 = 0000004,
		eB150 = 0000005,
		eB200 = 0000006,
		eB300 = 0000007,
		eB600 = 0000010,
		eB1200 = 0000011,
		eB1800 = 0000012,
		eB2400 = 0000013,
		eB4800 = 0000014,
		eB9600 = 0000015,
		eB19200 = 0000016,
		eB38400 = 0000017,
		eB57600 = 0010001,
		eB115200 = 0010002,
		eB230400 = 0010003,
		eB460800 = 0010004,
		eB500000 = 0010005,
		eB576000 = 0010006,
		eB921600 = 0010007,
		eB1000000 = 0010010,
		eB1152000 = 0010011,
		eB1500000 = 0010012,
		eB2000000 = 0010013,
		eB2500000 = 0010014,
		eB3000000 = 0010015,
		eB3500000 = 0010016,
		eB4000000 = 0010017
	};

#define DEVHANDLE int
#define COMMSETTINGS struct termios
#endif

#define DEFAULT_BAUDRATE eB19200

	/**
	 * @class CComAdapter
	 * @brief Class encapsules com port communication
	 * @author Stefan May
	 */
	class CComAdapter : public IDeviceAdapter
	{
 	public:

		/**
		 * Definition of function pointer to switch between buffered and unbuffered mode
		 */
		typedef unsigned long (CComAdapter::*FPtrReceive)(char*, unsigned long);

		/**
		 * standard constructor
		 */
		CComAdapter(const char* szDeviceName);

		/** Default Constructor. */
		CComAdapter();

		/**
		 * default destructor
		 */
		virtual ~CComAdapter();

		/**
		 * Set device name (will be considered in method open)
		 * @param szDeviceName name of device
		 */
		void setDeviceName(const char* szDeviceName);

		/**
		 * Get name of device
		 * @return name of device
		 */
		const char* getDeviceName() const;

		/**
		 * Set receive mode
		 * @param eMode receive mode
		 */
		void setReceiveMode(EnumReceiveMode eMode);

		/**
		 * Get receive mode
		 * @return receive mode
		 */
		EnumReceiveMode getReceiveMode() const;

		/**
		 * Open com port
		 * @return State of opening com port (success != 0)
		 */
		int openDevice();

		/**
		 * Close com port
		 * @return State of closing com port (success != 0)
		 */
		int closeDevice();

		/**
		 * Send message to device
		 * @param szMessage message
		 * @param ulLength length of message
		 */
		unsigned long send(const char* szMessage, unsigned long ulLength);

		/**
		 * Polymorphic wrapper function for unbuffered and buffered mode
		 * @param szAnswer message
		 * @param ulLength length of message to be read
		 * @return bytes read
		 */
		unsigned long receive(char* szAnswer, unsigned long ulLength);

		/**
		 * Read message unbuffered from device
		 * @param szAnswer message
		 * @param ulLength length of message to be read
		 * @return bytes read
		 */
		unsigned long receiveUnbuffered(char* szAnswer, unsigned long ulLength);

		/**
		 * Read message from device using a buffer.
		 * @param szAnswer message
		 * @param ulLength length of message to be read
		 * @return bytes read (if byte length < ulLength method returns 0. At next call try to complete the read)
		 */
		unsigned long receiveBuffered(char* szAnswer, unsigned long ulLength);

		/**
		 * Get type of adapter. The intention of this method is to allow
		 * someone to distinguish between different implementations.
		 * @return Type of connection as EnumAdapterType
		 */
		EnumAdapterType getAdapterType() const;

		/**
		 * Clear RX buffer of COM interface
		 * @return State of clearing buffer (success != 0)
		 */
		int clearReceiveBuffer();

		/**
		 * Clear TX buffer of COM interface
		 * @return State of clearing buffer (success != 0)
		 */
		int clearTransmissionBuffer();

		/**
		 * Set baudrate
		 * @param eBaudrate baudrate
		 */
		void setBaudrate(fair::EnumBaudrate eBaudrate);

		/**
		 * Get baudrate
		 * @return baudrate
		 */
		fair::EnumBaudrate getBaudrate() const;

		bool getRingIndicator();

		static void signal_handler_IO(int status)
		{
			printf("received SIGIO signal.\n");
		}

 	private:
		/**
		 * Private initialization routine
		 */
		void init();

		/**
		 * Type of this adapter.
		 */
		EnumAdapterType _type;

		/**
		 * Device handle
		 */
		DEVHANDLE _hCom;

		/**
		 * Device name
		 */
		std::string _sDeviceName;

		/**
		 * Baudrate
		 */
		fair::EnumBaudrate _eBaudrate;

		/**
		 * Receive mode
		 */
		EnumReceiveMode _eReceiveMode;

		/**
		 * Function pointer to receive function
		 */
		FPtrReceive _fptrReceive;

		/**
		 * Previous COM settings will be restored after deinitialization
		 */
		COMMSETTINGS _comSettings;

#ifdef WIN32
		/**
		 * Windows COM timeouts
		 */
		COMMTIMEOUTS _comTimeouts;
#endif

		/**
		 * bytes previously read in receiveBuffered
		 */
		unsigned long _ulBytesPrevRead;

		/**
		 * internal data buffer
		 */
		char _szBuffer[1024];

		/**
		 * Initialization already done?
		 */
		bool _bInit;
	};

}

#endif // #ifdef __COMADAPTER_H__
