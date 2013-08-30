/*
 *
 * libFAIR, Fraunhofer Autonomous Intelligent Robotics Library
 *
 * Copyright Fraunhofer Gesellschaft e. V., Munich, Germany
 *
 * The FAIR library [both binary and source code (if released)] is intellectual
 * property owned by Fraunhofer Gesellschaft and is protected by copyright;
 * the ownership remains with Fraunhofer Gesellschaft.
 *
 */

#ifndef IDEVICEADAPTER_H_
#define IDEVICEADAPTER_H_

namespace fair
{

	/**
	 * @brief Represents the adapter type of an IDeviceAdapter
	 *
	 * This enumeration represents the type of an IDeviceAdapter implementation. It
	 * will be returned by IDeviceAdapter::getAdapterType().
	 *
	 * Every implementation of IDeviceAdapter should be distinguishable by thi
	 * type. eUNKNOWN should not be used in any concrete implementation but is
	 * reserved for abstract classes.
	 *
	 * @author Alexander Grothkast
	 * @date July 2006
	 */
	enum EnumAdapterType
	{
		/**
		 * An unknown adapter type. Reserved for anstract calsses.
		 */
		eUNKNOWN = 1,
		/**
		 * Implements a fileadapter. Used by CFileAdapter.
		 */
		eFILE = 2,
		/**
		 * Implements a COM-adapter. Used by CComAdapter.
		 */
		eCOM = 3,
		/**
		 * Not used yet.
		 */
		eUSB = 4
	};

	/**
	 * @brief Describes the receive mode of a deviceadapter
	 *
	 * This enumeration represents the receive mode of a deviceadapter. An adapter
	 * can receive data either in a buffered (eBuffered) or an unbuffered
	 * (eUnbuffered) way:
	 * - In unbuffered mode the adapter tries to read up to a given number bytes of
	 * data. If it is only able to read less data it will return with the data read.
	 * That means an unbuffered receive returns any size of data between 0 bytes and
	 * the maximal bytes specified.
	 * - In buffered mode the adapter tries to read a given number bytes of data.
	 * If it is only able to read less data it will return no data. That means a
	 * buffered receive returns either 0 byte or the requested size of data.
	 *
	 * @author Alexander Grothkast
	 * @date July 2006
	 */
	enum EnumReceiveMode
	{
		/**
		 * Unbuffered receive mode.
		 */
		eUnbuffered,
		/**
		 * Buffered receive mode.
		 */
		eBuffered
	};

	/**
	 * @class IDeviceAdapter
	 * @brief Interface for communicating with devices
	 *
	 * This interface describes an adapter for communication with devices. It
	 * defines elementary operations to open and close a connection and to send and
	 * receive data. This interface hides how a device is connected and an
	 * implementation should encapsulate all operating system dependent
	 * implementation details (e.g. system calls) so the implementation of an device
	 * remains system independent.
	 *
	 * Every device is identified by its device name. The structure of a device
	 * name, what it looks like and how it is interpreted may depend on the
	 * implementation. Someone can think of an ordinary filename when a device is
	 * connected by a fileadapter, a device-file (e.g. /dev/ttyS0 or COM1) when
	 * using a COM-adapter. It is also imaginable that the devicename could be an
	 * IP-adress followed by a port number for an IP-adapter implementation.
	 *
	 * An implementation must allow setting a device name via the standard
	 * constructor IDeviceAdapteR(char* szDeviceName) when generating an instance or
	 * during lifetime via the methods setDeviceName() and getDeviceName().
	 *
	 * The methods openDevice() and closeDevice() must encapsulate all functionality
	 * to open or close a connection to the device. After an successful call of
	 * openDevice() send and receive operations must be possible.
	 *
	 * The methods send() and receive() should send or receive data to or from the
	 * device once it is connected. The receive() method must be implemented in a
	 * buffered and an unbuffered version. Someone can use the method
	 * setReceiveMode() to switch between the buffered and unbuffered receive modes.
	 * To indicate which mode is currently used the method getReceiveMode() returns
	 * an element of the EnumReceiveMode enumeration.
	 *
	 * As there is a big variety of configuration settings for different adapter
	 * types, this interface does not define any methods to change these settings.
	 * Such methods must be implemented in a way that fits the corresponding kind
	 * of settings which make sense in each case. To allow someone to determine
	 * which type of deviceadapter is used, and so to determine which methods to
	 * configure settings are available, the method getAdapterType() must be
	 * implemented.
	 *
	 * Although every implementation must allow an user to establish an at least
	 * rudimental connection only via the connect() method. That means that a call
	 * of connect() must not fail just because the user did not set some
	 * configuration settings which depend on the adapter type.
	 *
	 * @author Alexander Grothkast
	 * @date July 2006
	 */
	class IDeviceAdapter
	{
 	public:
		/**
		 * Standard constructor. An implementation of this constructor must do
		 * the same as the default constructor IDeviceAdapter(). In addition it
		 * must set the devicename szDeviceName in the same way a call of the
		 * method setDeviceName() with the same argument would do. So this means
		 * a call of this constructor must be equivalent to a call of the
		 * default constructor IDeviceAdapter() followed by a call of
		 * setDeviceName().
		 * @param szDeviceName Name of the device
		 * @post The object must be in the same state as if IDeviceAdapter() and
		 * setDeviceName(szDeviceName) would have been called instead.
		 */
		IDeviceAdapter(const char *szDeviceName)
		{
		}
		;

		/**
		 * Default constructor. This must initialize the object and must do the
		 * same as the standard constructor IDeviceAdapter(char* szDeviceName)
		 * except setting the device name.
		 * @post The object must be initialized
		 */
		IDeviceAdapter()
		{
		}
		;

		/**
		 * Default destructor. This should clean up.
		 * @post The object must be cleaned up.
		 */
		virtual ~IDeviceAdapter()
		{
		}
		;

		/**
		 * Open the device.
		 * @return !0 on success
		 */
		virtual int openDevice() = 0;

		/**
		 * Close the device.
		 * @return !0 on succes
		 */
		virtual int closeDevice() = 0;

		/**
		 * Send data.
		 * @param szBuffer Data to be sent
		 * @param ulLength Length of data in bytes
		 * @return Number of bytes sent or negative on error.
		 */
		virtual unsigned long send(const char *szBuffer, unsigned long ulLength) = 0;

		/**
		 * Receive data.
		 * @param szBuffer Buffer where received data will be stored
		 * @param ulLength Maximum number of bytes which will be received
		 * @return Number of bytes received or negative on error.
		 */
		virtual unsigned long receive(char *szBuffer, unsigned long ulLength) = 0;

		/**
		 * Get name of the device
		 * @return Devicename
		 */
		virtual const char *getDeviceName() const = 0;

		/**
		 * Set name of the device.
		 * If a new devicename is provided the device should be reopened to
		 * make sure that someone is using the new devicename.
		 * @param szDeviceName Devicename
		 */
		virtual void setDeviceName(const char *szDeviceName) = 0;

		/**
		 * Get type of adapter. The intention of this method is to allow
		 * someone to distinguish between different implementations.
		 * @return Type of connection as EnumAdapterType
		 */
		virtual EnumAdapterType getAdapterType() const = 0;

		/**
		 * Set receive mode
		 * @param eMode receive mode
		 */
		virtual void setReceiveMode(EnumReceiveMode eMode) = 0;

		/**
		 * Get receive mode
		 * @return receive mode
		 */
		virtual EnumReceiveMode getReceiveMode() const = 0;

 	private:

	};

}

#endif /*IDEVICEADAPTER_H_*/
