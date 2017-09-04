

#include "CError.h"


namespace VMC {
	
//////////////////////////////////////////////////////////////////////////////
///  @brief prints the Error to std::ostream
///  @returns std::ostream
//////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<(std::ostream& out, const CError& e) {
	out << std::showbase;
	if(e.getErrorString().size() != 0) {	
		out << "VMC Error: " << e.getErrorString();
	}else{
		out << e.getErrorType();
	}

//	if(e.getMethodName().size() != 0)
//		out << " | Method: \"" << e.getMethodName() << "\"";

	if(e.getcorrespondDataName().size() != 0)
		out << " | Data Name: " << e.getcorrespondDataName();

	if(e.getCorrespondMessage().isValidMessage() ) {
		out << " | CMD_GRP: " << std::hex << (int)(e.getCorrespondMessage()).getCommandGroup();
		out << " | CMD: " << std::hex << (int)(e.getCorrespondMessage()).getCommand();
		out << " | DATA: ";	
		std::vector<BYTE> DataFrame;
		(e.getCorrespondMessage()).getDataFrame(DataFrame);
		for(unsigned int i= 0; i< DataFrame.size(); i++) {
			out  << std::hex << (int)DataFrame[i] << " ";		
		}
	}
	
		
	out << " | Timestamp: " << std::dec << e.getTimestamp() << " ms";		
	return out;
}

}  // namespace VMC
