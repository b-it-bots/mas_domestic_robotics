//////////////////////////////////////////////////////////////////////////////
///  @file CError.h
///  @class VMC::CError
///  @brief An object of this class represent an error.
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////

#ifndef _CError_H_
#define _CError_H_

#include "enums.h"
#include "message.h"
#include "timestamp.h"
//#include "CData.h"
#include <string>
#include <iostream>

namespace VMC
{

	extern BYTE bumperTilt;

	class CError
	{
 	public:

//////////////////////////////////////////////////////////////////////////////
///  @brief standard constructor
//////////////////////////////////////////////////////////////////////////////
		CError()
		{
			m_sErrorType = "Unknown";
			bumperTilt = false;
		}

//////////////////////////////////////////////////////////////////////////////
///  @brief constructor
//////////////////////////////////////////////////////////////////////////////
		CError(const std::string& ErrorType)
				: m_sErrorType(ErrorType)
		{
			m_Timestamp.update();
		}

//////////////////////////////////////////////////////////////////////////////
///  @brief constructor
//////////////////////////////////////////////////////////////////////////////	
		CError(const std::string& ErrorType, const std::string& MethodName)
				: m_sErrorType(ErrorType),
				  m_sMethodName(MethodName)
		{
			m_Timestamp.update();
		}

//////////////////////////////////////////////////////////////////////////////
///  @brief constructor
//////////////////////////////////////////////////////////////////////////////	
		CError(const std::string& ErrorType, const std::string& MethodName, const std::string& correspondDataName)
				: m_sErrorType(ErrorType),
				  m_sMethodName(MethodName),
				  m_sCorrespondDataName(correspondDataName)
		{
			m_Timestamp.update();
		}

//////////////////////////////////////////////////////////////////////////////
///  @brief constructor
//////////////////////////////////////////////////////////////////////////////		
		CError(const std::string& ErrorType, const std::string& MethodName, const CMessage& correspondMessage)
				: m_sErrorType(ErrorType),
				  m_sMethodName(MethodName),
				  m_correspondMessage(correspondMessage)
		{
			m_Timestamp.update();
		}

//////////////////////////////////////////////////////////////////////////////
///  @brief constructor
//////////////////////////////////////////////////////////////////////////////
		CError(const std::string& ErrorType, const std::string& MethodName, const std::string& ErrorString, const CMessage& correspondMessage)
				: m_sErrorType(ErrorType),
				  m_sMethodName(MethodName),
				  m_sErrorString(ErrorString),
				  m_correspondMessage(correspondMessage)
		{
			m_Timestamp.update();
		}

//////////////////////////////////////////////////////////////////////////////
///  @brief standard destructor
//////////////////////////////////////////////////////////////////////////////
		~CError()
		{
		}
		;

//	void setErrorString(const std::string& ErrorString) {_ErrorString = ErrorString;}

//////////////////////////////////////////////////////////////////////////////
///  @brief returns the description of the error
///  @returns error type
//////////////////////////////////////////////////////////////////////////////
		std::string getErrorType() const
		{
			return m_sErrorType;
		}

//////////////////////////////////////////////////////////////////////////////
///  @brief returns the name of the method were the error occurred
///  @returns method name
//////////////////////////////////////////////////////////////////////////////
		std::string getMethodName() const
		{
			return m_sMethodName;
		}

//////////////////////////////////////////////////////////////////////////////
///  @brief return the error string of the VMC
///  @returns error string
//////////////////////////////////////////////////////////////////////////////
		std::string getErrorString() const
		{
			return m_sErrorString;
		}

//////////////////////////////////////////////////////////////////////////////
///  @brief return the Name of the VMC Parameter
///  @returns error string 
//////////////////////////////////////////////////////////////////////////////
		std::string getcorrespondDataName() const
		{
			return m_sCorrespondDataName;
		}

//////////////////////////////////////////////////////////////////////////////
///  @brief return the timestamp when the error occurred
///  @returns timestamp
//////////////////////////////////////////////////////////////////////////////
		long double getTimestamp() const
		{
			return m_Timestamp.getTime();
		}

//////////////////////////////////////////////////////////////////////////////
///  @brief return the corresponding message by which the error occurred
///  @returns CMessage object
//////////////////////////////////////////////////////////////////////////////
		CMessage getCorrespondMessage() const
		{
			return m_correspondMessage;
		}

		friend std::ostream& operator<<(std::ostream& out, const CError& e);

		void print();

 	private:

		std::string m_sErrorType;		//Description of the error

		std::string m_sMethodName;   	//Method were the error occurred

		std::string m_sCorrespondDataName;

		std::string m_sErrorString;  	//Error string of the VMC

		CTimestamp m_Timestamp;

		CMessage m_correspondMessage;

	};

}  // namespace VMC
#endif //_CError_H_
