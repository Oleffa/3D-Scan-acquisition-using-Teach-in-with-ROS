//////////////////////////////////////////////////////////////////////////////
///  @file CStorage.h
///  @class VMC::CStorage
///  @brief This class contain all parameters and runtime variables which belong to the whole contoller.
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////

#ifndef _CStorage_H_
#define _CStorage_H_

#include "CMotor.h"
#include "../SupportClasses/CData.h"
#include "../SupportClasses/CMultisend.h"
#include "../SupportClasses/CSendTwo.h"
#include "../SupportClasses/Enums.h"
#include "../SupportClasses/CChannel.h"
#include "../SupportClasses/CMessage.h"
#include <vector>
#include "../SupportClasses/commands.h"

namespace VMC {

class CStorage
{
public:		
	CStorage() {m_bInitialized = false;}
	
	~CStorage(){};
	
	bool Init(CTranslationLayer* Trans, BYTE* pNextRequestCommand);

	std::vector<VMC::CMotor> Motor;		

	CData BatteryVoltage;
	CData VelocityInputType;	
	CData VMCTimeout;
	CData VMCVersion;
	CData LoadParameters;
	CData SaveParameters;

	CMultisend MotorRPMs;
	CMultisend MotorPWMs;
	CSendTwo RobotVelocity;

	CData IOPortConfiguration;
//	CData ClearAllAbsolutRotations;  gone to multisend
	CMultisend ClearAllAbsolutRotations;
	CData DigitalIN;
//	CData DigitalOUT;  gone to multisend
	CMultisend DigitalOUT;
	CData AnalogInput1;
	CData AnalogInput2;
	
#ifdef REAL_TIME
  uint64_t getTimeStamp() { return timestamp;};
  void setTimeStamp(uint64_t ts) { timestamp = ts;};
private:
  uint64_t timestamp;
#else 
private:
#endif
	bool m_bInitialized;
	CTranslationLayer* m_pTrans;
};

}  // namespace VMC
#endif //_CStorage_H_
