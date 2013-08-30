#include "com_adapter.h"
#include <iostream>

using namespace std;

namespace fair
{

	CComAdapter::CComAdapter(const char* szDeviceName)
	{
		_sDeviceName = szDeviceName;
		_eBaudrate = DEFAULT_BAUDRATE;
		_hCom = 0;
		// default behavior is unbuffered, since other adapter
		// classes do not need to implement a buffered functionality
		_fptrReceive = &CComAdapter::receiveUnbuffered;
		_eReceiveMode = eUnbuffered;
		_type = eCOM;
		_ulBytesPrevRead = 0;
		_bInit = 0;
	}

	CComAdapter::CComAdapter()
	{
		_eBaudrate = DEFAULT_BAUDRATE;
		_hCom = 0;
		// default behavior is unbuffered, since other adapter
		// classes do not need to implement a buffered functionality
		_fptrReceive = &CComAdapter::receiveUnbuffered;
		_eReceiveMode = eUnbuffered;
		_ulBytesPrevRead = 0;
		_sDeviceName.erase();  // not "_sDeviceName.clear();" because of vc6
		_bInit = 0;
	}

	CComAdapter::~CComAdapter()
	{
		closeDevice();
	}

	void CComAdapter::init()
	{

		if (_bInit == 1)
			return;
		_bInit = 1;

#ifndef WIN32
		char cmd[255];

		if (_sDeviceName.empty())
			return;

		/*
		 setserial - get/set Linux serial port information
		 setserial [ -abqvVWz ] device [ parameter1 [ arg ] ] ...
		 setserial  is  a program designed to set and/or report the
		 configuration information associated with a  serial  port.
		 This information includes what I/O port and IRQ a particu
		 lar serial port is using, and whether or not the break key
		 should  be interpreted as the Secure Attention Key, and so
		 on.
		 baud_base baud_base
		 This option sets the base baud rate, which  is  the
		 clock frequency divided by 16.  Normally this value
		 is 115200, which is  also  the  fastest  baud  rate
		 which the UART can support.
		 spd_cust
		 Use  the  custom  divisor to set the speed when the
		 application requests 38.4kb.   In  this  case,  the
		 baud  rate is the baud_base divided by the divisor.
		 This parameter may be specified by a non-privileged
		 user.
		 divisor divisor
		 This  option sets the custom divison.  This divisor
		 will be used then the spd_cust option  is  selected
		 and  the serial port is set to 38.4kb by the appli
		 cation.  This parameter may be specified by a  non-
		 privileged user.
		 */
		snprintf(cmd, sizeof(cmd), "setserial -a %s", _sDeviceName.c_str());
		if (strncmp("/dev/ttyUSB", _sDeviceName.c_str(), 11) == 0)
			snprintf(cmd, sizeof(cmd), "setserial %s divisor 48 spd_cust", _sDeviceName.c_str());
		else
		{
			if (strncmp("/dev/ttyS", _sDeviceName.c_str(), 9) == 0)
				snprintf(cmd, sizeof(cmd), "setserial %s baud_base 500000 spd_cust divisor 1", _sDeviceName.c_str());
			else
				printf("error could not setserial device %s\n", _sDeviceName.c_str());
		}
		printf("%s\n", cmd);
		system(cmd);
		sleep(1);
#endif
	}

	void CComAdapter::setDeviceName(const char* szDeviceName)
	{
		_sDeviceName = szDeviceName;
		if (_eBaudrate == eB500000)
			init();
	}

	const char* CComAdapter::getDeviceName() const
	{
		return _sDeviceName.c_str();
	}

	void CComAdapter::setBaudrate(EnumBaudrate eBaudrate)
	{
		if (eBaudrate == eB500000)
			init();
		_eBaudrate = eBaudrate;
	}

	EnumBaudrate CComAdapter::getBaudrate() const
	{
		return _eBaudrate;
	}

	EnumAdapterType CComAdapter::getAdapterType() const
	{
		return _type;
	}

	void CComAdapter::setReceiveMode(EnumReceiveMode eMode)
	{
		_eReceiveMode = eMode;
		if (_eReceiveMode == eUnbuffered)
			_fptrReceive = &CComAdapter::receiveUnbuffered;
		else
			_fptrReceive = &CComAdapter::receiveBuffered;
	}

	EnumReceiveMode CComAdapter::getReceiveMode() const
	{
		return _eReceiveMode;
	}

	int CComAdapter::openDevice()
	{
		int nRetval = FAIR_EXIT_FAILURE;

		if (_sDeviceName.empty())
			return nRetval;

#ifdef WIN32

		DCB dcb;

		COMMTIMEOUTS timeouts;

		_hCom = CreateFile( _sDeviceName.c_str(),
				GENERIC_READ | GENERIC_WRITE,
				0,
				NULL,
				OPEN_EXISTING,
				FILE_ATTRIBUTE_NORMAL,		// no overlapped I/O
				NULL);// must be NULL for comm devices

		// store old comm settings and configure new ones
		nRetval = GetCommState(_hCom, &dcb);
		//if(nRetval)
		//{

		dcb.BaudRate = _eBaudrate;
		dcb.ByteSize = 8;
		dcb.Parity = NOPARITY;
		dcb.StopBits = ONESTOPBIT;
		//dcb.fDtrControl		= DTR_CONTROL_DISABLE;
		//dcb.fInX			= FALSE;
		dcb.fParity = FALSE;
		nRetval = SetCommState(_hCom, &dcb);

		if(nRetval)
		{

			// store old timeout settings and set new one
			nRetval = GetCommTimeouts (_hCom, &timeouts);
			if(nRetval)
			{
				// Set timeout to 0 to force that:
				// If a character is in the buffer, the character is read,
				// If no character is in the buffer, the function do not wait and returns immediatly
				timeouts.ReadIntervalTimeout = MAXDWORD;
				timeouts.ReadTotalTimeoutMultiplier = 0;
				timeouts.ReadTotalTimeoutConstant = 0;
				timeouts.WriteTotalTimeoutMultiplier = 0;
				timeouts.WriteTotalTimeoutConstant = 4;
				nRetval = SetCommTimeouts (_hCom, &timeouts);
			}
		}
		//}
#else

		COMMSETTINGS comSettingsNew;

		_hCom = open(_sDeviceName.c_str(), O_RDWR | O_NOCTTY);

		if (_hCom >= 0)
		{
			tcgetattr(_hCom, &_comSettings); /* save current port settings */
			/*
			 * this is deprecated
			 * bzero(&comSettingsNew,sizeof(comSettingsNew));
			 * using instead:
			 */
			memset(&comSettingsNew, 0, sizeof(comSettingsNew));

			comSettingsNew.c_cflag = (int) _eBaudrate | CS8 | CLOCAL | CREAD;
			/* set input mode raw */
			comSettingsNew.c_lflag = 0;
			comSettingsNew.c_iflag = 0;
			comSettingsNew.c_oflag = 0;
			comSettingsNew.c_cc[VTIME] = 0; /* inter-character timer unused */
			comSettingsNew.c_cc[VMIN] = 0; /* nonblocking read */

			tcflush(_hCom, TCIFLUSH);
			tcsetattr(_hCom, TCSANOW, &comSettingsNew);
			nRetval = FAIR_EXIT_SUCCESS;
		}
		else
			nRetval = FAIR_EXIT_FAILURE;

#endif

		return nRetval;
	}

	int CComAdapter::closeDevice()
	{
		int nRetval = FAIR_EXIT_FAILURE;
		if (_hCom != 0)
#ifdef WIN32
			if(CloseHandle(_hCom)) nRetval = FAIR_EXIT_SUCCESS;
#else
			if (close(_hCom))
				nRetval = FAIR_EXIT_SUCCESS;
#endif
		_hCom = 0;
		return nRetval;
	}

	unsigned long CComAdapter::send(const char* szMessage, unsigned long ulLength)
	{
		if (_hCom == 0)
			return 0;

		unsigned long ulBytesWritten;
#ifdef WIN32
		WriteFile( _hCom, szMessage, ulLength, &ulBytesWritten, NULL);
#else
		ulBytesWritten = write(_hCom, szMessage, ulLength);
#endif
		//printf("Bytes written: %d\n",ulBytesWritten);
		return ulBytesWritten;
	}

	unsigned long CComAdapter::receive(char* szAnswer, unsigned long ulLength)
	{
		return (this->*_fptrReceive)(szAnswer, ulLength);
	}

	unsigned long CComAdapter::receiveUnbuffered(char* szAnswer, unsigned long ulLength)
	{
		if (_hCom == 0)
			return 0;

		unsigned long ulBytesRead;
#ifdef WIN32
		ReadFile ( _hCom, szAnswer, ulLength, &ulBytesRead, NULL);
#else
		ulBytesRead = read(_hCom, szAnswer, ulLength);
#endif
		//printf("Bytes read: %d\n",ulBytesRead);
		return ulBytesRead;
	}

	unsigned long CComAdapter::receiveBuffered(char* szAnswer, unsigned long ulLength)
	{
		if (_hCom == 0)
			return 0;

		unsigned long ulBytesRead = 0;
		unsigned long ulBytesToRead = ulLength - _ulBytesPrevRead;
		unsigned long ulBytesReadBuf = 0;
		//cout << "Len: " << ulLength << endl;
		//cout << "ToRead: " << ulBytesToRead << endl;

#ifdef WIN32
		ReadFile ( _hCom, &_szBuffer[_ulBytesPrevRead], ulBytesToRead, &ulBytesReadBuf, NULL);
#else
		ulBytesReadBuf = read(_hCom, &_szBuffer[_ulBytesPrevRead], ulBytesToRead);
#endif
		//cout << "Read: " << ulLength << endl;
		if (ulBytesReadBuf == ulBytesToRead)
		{
			ulBytesRead = (unsigned short) ulLength;
			memcpy(szAnswer, _szBuffer, ulLength);
			_ulBytesPrevRead = 0;
		}
		else if (ulBytesReadBuf > ulBytesToRead)
		{
			ulBytesRead = (unsigned short) ulLength;
			memcpy(szAnswer, _szBuffer, ulLength);
			unsigned long ulTooMany = ulBytesReadBuf - ulBytesToRead;
			char* pucTmp = new char[ulTooMany];
			memcpy(pucTmp, &_szBuffer[_ulBytesPrevRead], ulTooMany);
			memcpy(_szBuffer, pucTmp, ulTooMany);
			_ulBytesPrevRead = ulTooMany;
			delete pucTmp;
		}
		else
		{
			_ulBytesPrevRead += ulBytesReadBuf;
		}

		return ulBytesRead;
	}

	int CComAdapter::clearReceiveBuffer()
	{
#ifdef WIN32
		if(_hCom != INVALID_HANDLE_VALUE)
		{
			return PurgeComm(_hCom, PURGE_RXCLEAR);
		}
#else
		if (_hCom != 0)
		{
			return tcflush(_hCom, TCIFLUSH);
		}
#endif
		return FAIR_EXIT_SUCCESS;
	}

	int CComAdapter::clearTransmissionBuffer()
	{
#ifdef WIN32
		if(_hCom != INVALID_HANDLE_VALUE)
		{
			return PurgeComm(_hCom, PURGE_TXCLEAR);
		}
#else
		if (_hCom != 0)
		{
			return tcflush(_hCom, TCOFLUSH);
		}
#endif
		return FAIR_EXIT_SUCCESS;
	}

	bool CComAdapter::getRingIndicator()
	{
		int signalState;
//	cout << "in getRingIndicator" << endl;
		ioctl(_hCom, TIOCMIWAIT, TIOCM_DSR);

//	cout << "TIOCMIWAIT getRingIndicator" << endl;	

		ioctl(_hCom, 21525, &signalState);
//	ioctl(_hCom, TIOCM_DSR , &signalState);
//256
//	cout << "signal state: " << signalState << endl;
//	cout << TIOCM_DSR << endl;
//	cout << TIOCM_RNG << endl;
//	if (signalState & TIOCM_DSR) {
		if (signalState & 256)
		{
			return true;
		}
		return false;
	}

}
