//////////////////////////////////////////////////////////////////////////////
///  @file CTranslationLayer.h
///  @class VMC::CTranslationLayer
///  @brief Translates from the binary message string to a CMessage object and vice versa.
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////
#ifndef _CTranslationLayer_H_
#define _CTranslationLayer_H_

#include "CCommunicationLayer.h"
#include "../SupportClasses/CMessage.h"
#include "../SupportClasses/Enums.h"
#include "../SupportClasses/CError.h"
#include <string>
#include <list>
#include <iostream>

namespace VMC {
class CTranslationLayer
{
public:
	CTranslationLayer();
	~CTranslationLayer(){};

	bool sendMessage(const CMessage& Message);		
#ifdef REAL_TIME
  bool receiveMessage(uint64_t &timestamp);
#else 
	bool receiveMessage();		
#endif
	bool findMessage(CMessage& NewMessage);

	bool openDevice();
	bool closeDevice();
private:		

	static const BYTE sm_PStart;
	static const BYTE sm_PEnd;
	static const BYTE sm_Quote;
	
	void appendByte(std::string& MessageString, BYTE byte);
	std::string m_sReceiveBuffer;		
	CCommunicationLayer m_Com;
	
	BYTE m_SendPacketCounter;
	BYTE m_ReceivePacketCounter;
	
	bool m_bReceivedMessage;
	
	friend class CvmcAPI;
};

}  // namespace VMC
#endif //_CTranslationLayer_H_
