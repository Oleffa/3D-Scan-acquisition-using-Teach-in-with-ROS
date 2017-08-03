#ifndef _Config_H_
#define _Config_H_

//#define ICONNECT

#ifdef ICONNECT
	#include <afxwin.h>			//is only needed when API is used for ICONNECT module
#endif

namespace VMC {


// defines if the program will compile for Windows (WIN32), 
// Mac OS X (MACOSX) or Linux (no definition)
//#define WIN32 1
//#define MACOSX 1
	
// defines if the debug mode is active or not 
// (for normal operation it is recommended to disable it)
//#define VMC_DEBUG 1

typedef unsigned char BYTE;


}  // namespace VMC
#endif //_Config_H_
