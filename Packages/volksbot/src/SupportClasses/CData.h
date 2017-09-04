//////////////////////////////////////////////////////////////////////////////
///  @file CData.h
///  @class VMC::CData
///  @brief Stores one Parameter of the VMC.
///  @note In this class a whole VMC parameter or runtime variable is encapsuled.
///  @author Jan Paulus
///  @version 0.1
///  @date 16.10.2006
//////////////////////////////////////////////////////////////////////////////


#ifndef _CData_H_
#define _CData_H_

#include "CChannel.h"
#include "CMessage.h"
#include "Enums.h"
#include "CTimestamp.h"
#include "../LayerClasses/CTranslationLayer.h"
#include "CError.h"
#include <string>
#include <list>
#include <sstream>

namespace VMC {

extern std::list<CError> VMC_Errors;

class CData
{
public:		

	CData() {m_bInitialized = false;}
					
	~CData(){};
	
	bool Init(	CTranslationLayer* Trans,
				BYTE* pNextRequestCommand,
				const BYTE nCommandGroup, 
				const BYTE nCommand,
				const EnumDataType DataType,
				const CChannel& MotorChannel,			
				const std::string& cName,
				const double dValue, 
				const double dMaximum,
				const double dMinimum,
				const EnumUnitPrefix UnitPrefix = one,
				const EnumUnit Unit = none);

	double getValue() const;
	bool Update() const;		
	bool Set(const double Value) const;
	
	EnumDataType getDataType() const;	
		
	void updateTimestamp();		
	long double getTimestamp() const;		
	
	bool setRange(const double Maximum,const double Minimum);		
	double getMaximum() const;		
	double getMinimum() const;		
	
	std::string getName() const;	
	
	unsigned int getMotorChannel() const;
	
	void setUnitPrefix(const EnumUnitPrefix& UnitPrefix);		
	std::string getUnitPrefix() const;
	
	void setUnit(const EnumUnit& _Unit);		
	std::string getUnit() const;		
	
	friend std::ostream& operator<<(std::ostream& out,const CData& d);

	bool giveInitState();	

private:
	bool setValue(const double Value);
	bool setValue(const std::vector<BYTE>& DataFrame);	
	
	friend class CvmcAPI;

	bool m_bInitialized;
	CTranslationLayer* m_pTrans;
	BYTE* m_pNextRequestCommand;
	BYTE m_CommandGroup;		
	BYTE m_Command;
	EnumDataType m_DataType;
	CChannel m_MotorChannel;		
	std::string m_sName;	
	double m_dValue;
	double m_dMaximum;		
	double m_dMinimum;
	EnumUnitPrefix m_UnitPrefix;
	EnumUnit m_Unit;	
	CTimestamp m_Timestamp;
};

}  // namespace VMC

#endif //_CData_H_

//@}

