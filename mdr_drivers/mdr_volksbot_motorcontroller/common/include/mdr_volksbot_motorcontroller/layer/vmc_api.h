//////////////////////////////////////////////////////////////////////////////
///  @file CvmcAPI.h
///  @class VMC::CvmcAPI
///  @brief This is a part of the User Interface Layer. The user access to the API is only through an object of this class.
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////

#ifndef _CvmcAPI_H_
#define _CvmcAPI_H_

#include <list>
#include <string>
#include <iostream>
#include <sstream>

#include "translation_layer.h"
#include "communication_layer.h"
#include "support/enums.h"
#include "support/message.h"
#include "support/request.h"
#include "support/error.h"
#include "storage/storage.h"
#include "support/commands.h"

namespace VMC
{

	class CVmcApi
	{
 	public:
		CVmcApi();
		~CVmcApi()
		{
		}
		;

		bool configRequestMessage(unsigned int nRequestNo, bool bActive);
		bool printRequestMessage();

		CStorage& useVMC();

		bool selectHardwareAdapter(const EnumAdapter& Adapter);
		bool selectDevice(const char* DeviceName)
		{
			return m_Trans.m_Com.selectDevice(DeviceName);
		}
		bool openDevice();
		bool closeDevice();
		void printAllErrors();
		std::string getVersion()
		{
			return m_VMC_API_Version;
		}
		std::vector<CRequest>* getRequests()
		{
			return &this->m_Request;
		}
		;

		int WhichBumper();

 	private:
		bool getNextRequestCommand();
		bool storeMessageData(const CMessage& Message);
		bool interpreteBasicSystemCommands(const CMessage& Message);
		bool interpreteConfigurationResponse(const CMessage& Message);
		bool interpreteStatusResponse(const CMessage& Message);
		bool interpreteErrorMessages(const CMessage& Message);
		bool interpreteControllerConfigurationResponse(const CMessage& Message);
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@bumper
		CTranslationLayer m_Trans;
		CStorage m_VMC;
		CMessage m_ReceivedMessage;
		std::vector<CRequest> m_Request;
		std::vector<CRequest>::iterator m_RequestIterator;
		BYTE NextRequestCommand;

		std::string m_VMC_API_Version;
	};

}  // namespace VMC
#endif //_CvmcAPI_H_
