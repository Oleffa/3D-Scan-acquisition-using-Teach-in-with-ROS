//////////////////////////////////////////////////////////////////////////////
///  @file CStorage.cpp
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////


#include "CStorage.h"


namespace VMC {
	

bool CStorage::Init(CTranslationLayer* Trans, BYTE* pNextRequestCommand) {
	
	if(m_bInitialized)
		return false;
	
	m_pTrans = Trans;
	CChannel Channel0(0);
	CMotor* MotorPointer;
	
	BatteryVoltage.Init(	
		m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		_CMDGRP_MOTOR_STATUSIN_, _SREG_BATTERY_,								//Command Group / Command
		SignedInteger, Channel0, "BatteryVoltage",	//Data Type / Motor Channel / Name
		0.0, 32767.0, 0.0, milli, volt);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

	VMCTimeout.Init(	
		m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		_CMDGRP_MOTOR_CONFIN_, _MCMD_TIMEOUT_,								//Command Group / Command
		SignedInteger, Channel0, "VMCTimeout",	//Data Type / Motor Channel / Name
		0.0, _TIMEOUT_, 0.0, milli, second);		//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		
	VMCVersion.Init(	
		m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		_CMDGRP_BASE_, 0x10,			 					//Command Group / Command
		String, Channel0, "VMCVersion",			//Data Type / Motor Channel / Name
		0.0, 0.0, 0.0, one, none);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit
		
	VelocityInputType.Init(	
		m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		_CMDGRP_MOTOR_CONFIN_, 0x11,			 					//Command Group / Command
		SignedInteger, Channel0, "VelocityInputType",			//Data Type / Motor Channel / Name
		0.0, 1.0, 0.0, one, none);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

	LoadParameters.Init(	
		m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		_CMDGRP_MOTOR_CONFIN_, _MCMD_LOAD_CONFIG_,			 					//Command Group / Command
		SignedInteger, Channel0, "LoadParameters",			//Data Type / Motor Channel / Name
		1.0, _CONFIG_, 1.0, one, none);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

	SaveParameters.Init(	
		m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		_CMDGRP_MOTOR_CONFIN_, _MCMD_SAVE_CONFIG_,				 				//Command Group / Command
		SignedInteger, Channel0, "SaveParameters",			//Data Type / Motor Channel / Name
		1.0, _CONFIG_, 1.0, one, none);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

	MotorPWMs.Init(	
		m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		_CMDGRP_MOTOR_CTRL_, _SET_ALL_PWM_, SignedInteger);				//Command Group / Command / Data Type
		
	MotorRPMs.Init(	
		m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		_CMDGRP_MOTOR_CTRL_, _SET_ALL_RPM_, SignedInteger);				//Command Group / Command / Data Type

	RobotVelocity.Init(	
		m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		_CMDGRP_MOTOR_CTRL_, 0x30, SignedInteger);				//Command Group / Command / Data Type
	//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

	IOPortConfiguration.Init(	
		m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		_CMDGRP_MOTOR_CONFIN_, _MCMD_IO_PORT_CONFIG_ ,				 				//Command Group / Command
		SignedInteger, Channel0, "IOPortConfiguration",			//Data Type / Motor Channel / Name
		0.0, 31.0, 0.0, one, none);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

	ClearAllAbsolutRotations.Init(	
		m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		_CMDGRP_MOTOR_CTRL_, _SET_CLEAR_ALL_TICKS_ABS_, SignedInteger);	 				//Command Group / Command

	DigitalOUT.Init(	
		m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		_CMDGRP_MOTOR_CTRL_, _SET_DIGITAL_OUT_, SignedInteger);				//Command Group / Command / Data Type

	DigitalIN.Init(	
		m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		_CMDGRP_MOTOR_STATUSIN_, _SREG_DIGITAL_IN_,			 					//Command Group / Command
		SignedInteger, Channel0, "DigitalIN",	//Data Type / Motor Channel / Name
		0.0, 31.0, 0.0, one, none);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

	AnalogInput1.Init(	
		m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		_CMDGRP_MOTOR_STATUSIN_, _SREG_ANALOG1_IN_,			 					//Command Group / Command
		SignedInteger, Channel0, "AnalogInput1",	//Data Type / Motor Channel / Name
		111.0, 1024.0, 0.0, one, none);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit

	AnalogInput2.Init(	
		m_pTrans, pNextRequestCommand,			//Pointer to the Translation Layer / Pointer to next Request Command
		_CMDGRP_MOTOR_STATUSIN_, _SREG_ANALOG2_IN_,			 					//Command Group / Command
		SignedInteger, Channel0, "AnalogInput2",	//Data Type / Motor Channel / Name
		333.0, 1024.0, 0.0, one, none);				//Value / Maximum Value / Minimum Value / Unit Prefix, Unit


	CChannel Channel(1);
	for(unsigned int i=0; i < Channel0.getMaxChannels(); i++) {
		MotorPointer = new CMotor;
		Channel.set(i);
		MotorPointer->Init(m_pTrans, pNextRequestCommand, Channel);
		Motor.push_back(*MotorPointer);
		delete MotorPointer;
	}
	m_bInitialized = true;
	return true;
}

}  // namespace VMC
