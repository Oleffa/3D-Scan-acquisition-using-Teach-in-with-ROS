#ifndef COMADAPTER_H__
#define COMADAPTER_H__

#include <stdio.h>
#include <string.h>
//#include "fair/core/io/IDeviceAdapter.h"			//changed by Jan
#include "IDeviceAdapter.h"
//#include "fair/core/base/common.h"				//changed by Jan
#include "common.h"

#ifndef WIN32
	#include <fcntl.h>
	#include <termios.h>
#elif REAL_TIME
#include <rtdm/rtserial.h>
#endif

namespace fair
{

	
#ifdef WIN32
	/**
	 * Possible COM port baudrates
	 */
	enum EnumBaudrate { eB9600 = 9600, eB19200 = 19200, eB38400 = 38400, eB57600 = 57600, eB500000 = 500000};

	#define DEVHANDLE HANDLE
	#define COMMSETTINGS DCB

#elif REAL_TIME
	/**
	 * Possible COM port baudrates
	 */
	enum EnumBaudrate { eB9600 = 9600, eB19200 = 19200, eB38400 = 38400, eB57600 = 57600, eB500000 = 500000 };

	#define DEVHANDLE int
	//#define COMMSETTINGS struct rtser_config // doesnt work, so we'll circumvent for now
	#define COMMSETTINGS struct termios
#else
	/**
	 * Possible COM port baudrates
	 */
	enum EnumBaudrate { eB9600 = 0000015, eB19200 = 0000016, eB38400 = 0000017, eB57600 = 0010001, eB500000 = 0000017 };

	#define DEVHANDLE int
	#define COMMSETTINGS struct termios
#endif

#define DEFAULT_BAUDRATE eB19200



/**
 * @class CComAdapter
 * @brief Class encapsules com port communication
 * @author Stefan May
 */
class CComAdapter : public IDeviceAdapter {
public:

	typedef unsigned long (CComAdapter::*FPtrReceive)(char*, unsigned long);
	
	/**
	 * standard constructor
	 */
	CComAdapter(char* szDeviceName);
	
	/** Default Constructor. */
	CComAdapter();
	
	/**
	 * default destructor
	 */
	virtual ~CComAdapter();
	
	/**
	 * Set device name (will be considered in method open)
	 * @param szDeviceName name of device
	 */
	void setDeviceName(char* szDeviceName);
	
	/**
	 * Get name of device
	 * @return name of device
	 */
	char* getDeviceName();

	
	/**
	 * Set receive mode
	 * @param eMode receive mode
	 */
	void setReceiveMode(EnumReceiveMode eMode);
	
	/**
	 * Get receive mode
	 * @return receive mode
	 */
	EnumReceiveMode getReceiveMode();
	
	/**
	 * Open com port
	 * @return State of opening com port (success != 0)
	 */
	int openDevice();
	
	/**
	 * Close com port
	 * @return State of closing com port (success != 0)
	 */
	int closeDevice();
	
	/**
	 * Send message to device
	 * @param szMessage message
	 * @param ulLength length of message
	 */
	unsigned long send(char* szMessage, unsigned long ulLength);
	
	/**
	 * Polymorphic wrapper function for unbuffered and buffered mode
	 * @param szAnswer message
	 * @param ulLength length of message to be read
	 * @return bytes read
	 */
	unsigned long receive(char* szAnswer, unsigned long ulLength);

	/**
	 * Read message unbuffered from device
	 * @param szAnswer message
	 * @param ulLength length of message to be read
	 * @return bytes read
	 */	
	unsigned long receiveUnbuffered(char* szAnswer, unsigned long ulLength);
	
	/**
	 * Read message from device using a buffer.
	 * @param szAnswer message
	 * @param ulLength length of message to be read
	 * @return bytes read (if byte length < ulLength method returns 0. At next call try to complete the read)
	 */
	unsigned long receiveBuffered(char* szAnswer, unsigned long ulLength);
	
	/**
	 * Get type of adapter. The intention of this method is to allow
	 * someone to distinguish between different implementations.
	 * @return Type of connection as EnumAdapterType
	 */
	EnumAdapterType getAdapterType();
	
	/**
	 * Clear RX buffer of COM interface
	 * @return State of clearing buffer (success != 0)
	 */
	int clearReceiveBuffer();
	
	/**
	 * Clear TX buffer of COM interface
	 * @return State of clearing buffer (success != 0)
	 */
	int clearTransmissionBuffer();

	/**
	 * Set baudrate
	 * @param eBaudrate baudrate
	 */
	void setBaudrate(fair::EnumBaudrate eBaudrate);
	
	/**
	 * Get baudrate
	 * @return baudrate
	 */
	fair::EnumBaudrate getBaudrate();
	
private:
	/**
	 * Private initialization routine
	 */
	void init();

	/**
	 * Type of this adapter.
	 */
	EnumAdapterType _type;
		
	/**
	 * Device handle
	 */
	DEVHANDLE _hCom;
	
	/**
	 * Device name
	 */
	char* _szDeviceName;
	
	/**
	 * Baudrate
	 */
	fair::EnumBaudrate _eBaudrate;
	
	/**
	 * Receive mode
	 */
	EnumReceiveMode _eReceiveMode;
	
	/**
	 * Function pointer to receive function
	 */
	FPtrReceive _fptrReceive;
	
	/**
	 * Previous COM settings will be restored after deinitialization
	 */
	COMMSETTINGS _comSettings;
	
	#ifdef WIN32
		/**
		 * Windows COM timeouts
		 */
		COMMTIMEOUTS _comTimeouts;
	#endif
	
	/**
	 * bytes previously read in receiveBuffered
	 */
	unsigned long _ulBytesPrevRead;
	
	/**
	 * internal data buffer
	 */
	char _szBuffer[1024];
};

}

#endif // #ifdef __COMADAPTER_H__
