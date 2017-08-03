//////////////////////////////////////////////////////////////////////////////
///  @file CMotor.h
///  @class VMC::CMotor
///  @brief This class contain all parameters and runtime variables which belong to an motor.
///  @author Jan Paulus
///  @version 0.1
///  @date 23.10.2006
//////////////////////////////////////////////////////////////////////////////

#ifndef _CMotor_H_
#define _CMotor_H_

#include "../SupportClasses/Enums.h"
#include "../SupportClasses/CData.h"
#include "../SupportClasses/CChannel.h"
#include "../SupportClasses/commands.h"

namespace VMC {

class CMotor
{
public:		
	CMotor() {m_bInitialized = false;}
	~CMotor(){};
	bool Init(CTranslationLayer* Trans, BYTE* pNextRequestCommand, CChannel ch);
	
	CData MotorState;		
	CData Encoderfalse;		
	CData MotorOberheat;		
	CData BridgeOverheat;
	
	CData VelocityControllerState;
	CData CurrentControllerState;

	CData PositivRamp;
	CData NegativRamp;
	CData DeadBand;
	
	CData VelocityControllerLinearPart;		
	CData VelocityControllerDifferentialPart;		
	CData VelocityControllerIntegralPart;		

	CData CurrentControllerLinearPart;		
	CData CurrentControllerDifferentialPart;		
	CData CurrentControllerIntegralPart;	

	CData MaximumCurrent;		
	CData NominalCurrent;		
	CData ActualCurrent;		
	CData ActualPowerOutput;
	
	CData WheelRadius;		
	CData GearRatio;		
	CData AxeLength;		
	CData EncoderTicks;
	
	CData TorqueConstant;
	
	CData AbsolutRotations;
	CData ClearOneAbsolutRotations;
	CData EncoderTicksRelativ;
	
	CData ActualWindingTemperature;		
	CData ActualChassisTemperature;		
	
//	CData ThermalResistanceHousing;
	CData ThermalResistanceHousingNumerator;
	CData ThermalResistanceHousingDenominator;	
		
//	CData ThermalResistanceWinding;	
	CData ThermalResistanceWindingNumerator;	
	CData ThermalResistanceWindingDenominator;	
		
	CData ThermalTimeConstantWinding;		
	CData ThermalTimeConstantMotor;		
	CData MaximumTemperature;		
	CData NominalTemperature;
	CData AmbientTemperature;
	
	CData MaximumRPM;		
	CData NominalRPM;		
	CData DesiredRPM;		
	CData ActualRPM;
	
	CData MaximumPWM;		
	CData NominalPWM;		
	CData DesiredPWM;		
	CData ActualPWM;

private:
	bool m_bInitialized;
	CTranslationLayer* m_pTrans;
	
	CChannel m_Channel;
};

}  // namespace VMC
#endif //_CMotor_H_
