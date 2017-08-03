#include "CComAdapter.h"
#include "../SupportClasses/Config.h"
#ifdef REAL_TIME
#include <rtdm/rtserial.h>
#include <native/timer.h>
#endif 

namespace fair
{

CComAdapter::CComAdapter(char* szDeviceName)
{
	_szDeviceName = szDeviceName;
	_eBaudrate = DEFAULT_BAUDRATE;
	_hCom = -1;
	// default behavior is unbuffered, since other adapter
	// classes do not need to implement a buffered functionality
	_fptrReceive = &CComAdapter::receiveUnbuffered;
	_eReceiveMode = eUnbuffered;
	_type = eCOM;
}

CComAdapter::CComAdapter()
{
	_szDeviceName = NULL;
	_eBaudrate = DEFAULT_BAUDRATE;
	_hCom = -1;
	// default behavior is unbuffered, since other adapter
	// classes do not need to implement a buffered functionality
	_fptrReceive = &CComAdapter::receiveUnbuffered;
	_eReceiveMode = eUnbuffered;
	_ulBytesPrevRead = 0;
}

CComAdapter::~CComAdapter()
{
	 closeDevice();
}

void CComAdapter::init()
{
#ifndef WIN32
#ifndef MACOSX
#ifndef REAL_TIME
  char cmd[255];
  printf("CComAdapter::init()\n");
  if (_szDeviceName == NULL) return;
  
  /*
    setserial - get/set Linux serial port information
    setserial [ -abqvVWz ] device [ parameter1 [ arg ] ] ...
    setserial  is  a program designed to set and/or report the
    configuration information associated with a  serial  port.
    This information includes what I/O port and IRQ a particu
    lar serial port is using, and whether or not the break key
    should  be interpreted as the Secure Attention Key, and so
    on.
    baud_base baud_base
    This option sets the base baud rate, which  is  the
    clock frequency divided by 16.  Normally this value
    is 115200, which is  also  the  fastest  baud  rate
    which the UART can support.
    spd_cust
    Use  the  custom  divisor to set the speed when the
    application requests 38.4kb.   In  this  case,  the
    baud  rate is the baud_base divided by the divisor.
    This parameter may be specified by a non-privileged
    user.
    divisor divisor
    This  option sets the custom divison.  This divisor
    will be used then the spd_cust option  is  selected
    and  the serial port is set to 38.4kb by the appli
    cation.  This parameter may be specified by a  non-
    privileged user.
  */
  snprintf(cmd,sizeof(cmd),"setserial -a %s",_szDeviceName);  
  if (strncmp("/dev/ttyUSB",_szDeviceName,11)==0) 
      snprintf(cmd,sizeof(cmd),"setserial %s divisor 48 spd_cust",_szDeviceName);
  else {
    if (strncmp("/dev/ttyS",_szDeviceName,9)==0) 
	 //snprintf(cmd,sizeof(cmd),"setserial %s baud_base 500000 spd_cust divisor 1",_szDeviceName); 
   // Changed by Jan Elseberg:
	 snprintf(cmd,sizeof(cmd),"setserial %s baud_base 115200 spd_normal divisor 0",_szDeviceName);  
    else printf ("error could not setserial device %s\n",_szDeviceName);
  }
  system(cmd);
  sleep(1);
#endif
#endif
#endif
}

void CComAdapter::setDeviceName(char* szDeviceName)
{
	_szDeviceName = szDeviceName;
	init();
}

char* CComAdapter::getDeviceName()
{
	return _szDeviceName;
}

void CComAdapter::setBaudrate(EnumBaudrate eBaudrate)
{
	if(eBaudrate==eB500000) init();
	_eBaudrate = eBaudrate;
}

EnumBaudrate CComAdapter::getBaudrate()
{
	return _eBaudrate;
}

EnumAdapterType CComAdapter::getAdapterType() {
	return _type;
}

void CComAdapter::setReceiveMode(EnumReceiveMode eMode)
{
	_eReceiveMode = eMode;
	if(_eReceiveMode == eUnbuffered)
		_fptrReceive = &CComAdapter :: receiveUnbuffered;
	else
		_fptrReceive = &CComAdapter :: receiveBuffered;	
}

EnumReceiveMode CComAdapter::getReceiveMode()
{
	return _eReceiveMode;	
}
	
	
int CComAdapter::openDevice()
{
	int nRetval = 0;
	
	if (_szDeviceName == NULL) return nRetval;

#ifdef WIN32

	DCB dcb;
	
	COMMTIMEOUTS timeouts;  
   
	_hCom = CreateFile(	_szDeviceName,
						GENERIC_READ | GENERIC_WRITE,
						0,
						NULL,
						OPEN_EXISTING,
						FILE_ATTRIBUTE_NORMAL,		// no overlapped I/O
						NULL);	// must be NULL for comm devices

	// store old comm settings and configure new ones
	nRetval = GetCommState(_hCom, &dcb);
	//if(nRetval)
	//{
		
		dcb.BaudRate		= _eBaudrate;
		dcb.ByteSize		= 8;
		dcb.Parity			= NOPARITY;
		dcb.StopBits		= ONESTOPBIT;
		//dcb.fDtrControl		= DTR_CONTROL_DISABLE;
		//dcb.fInX			= FALSE;
		dcb.fParity			= FALSE;
		nRetval = SetCommState(_hCom, &dcb);

		if(nRetval)
		{
			
			// store old timeout settings and set new one
			nRetval = GetCommTimeouts (_hCom, &timeouts);
			if(nRetval)
			{
				// Set timeout to 0 to force that:
				// If a character is in the buffer, the character is read,
				// If no character is in the buffer, the function do not wait and returns immediatly
				timeouts.ReadIntervalTimeout			= MAXDWORD;
				timeouts.ReadTotalTimeoutMultiplier		= 0;
				timeouts.ReadTotalTimeoutConstant		= 0;
				timeouts.WriteTotalTimeoutMultiplier	= 0;
				timeouts.WriteTotalTimeoutConstant		= 4;
				nRetval = SetCommTimeouts (_hCom, &timeouts);
			}
		}
	//}
#elif REAL_TIME  // xenomai

  struct rtser_config comSettingsNew;
  //COMMSETTINGS comSettingsNew;
	
	_hCom = rt_dev_open(_szDeviceName, O_RDWR | O_NOCTTY );

	printf("%s %d %s\n", _szDeviceName, _hCom,  strerror(-_hCom));
	
	if (_hCom >= 0)
	{
		/* save current port settings */
    rt_dev_ioctl(_hCom, RTSER_RTIOC_GET_CONFIG, &comSettingsNew);
    comSettingsNew.config_mask       =  RTSER_SET_BAUD | RTSER_SET_PARITY | 
                                        RTSER_SET_DATA_BITS | 
                                        RTSER_SET_STOP_BITS | 
                                        RTSER_SET_HANDSHAKE | 
                                        RTSER_SET_TIMEOUT_RX |
                                        RTSER_SET_TIMEOUT_TX | 
                                        RTSER_SET_FIFO_DEPTH | 
                                        RTSER_SET_TIMEOUT_EVENT |
                                        RTSER_SET_EVENT_MASK | 
                                        RTSER_SET_TIMESTAMP_HISTORY;

//    comSettingsNew.baud_rate         = 57600;//115200;
    comSettingsNew.baud_rate         = _eBaudrate;
    comSettingsNew.parity            = RTSER_NO_PARITY;
    comSettingsNew.data_bits         = RTSER_8_BITS;   
    comSettingsNew.stop_bits         = RTSER_1_STOPB;
    comSettingsNew.handshake         = RTSER_NO_HAND;
    comSettingsNew.fifo_depth        = 0;//0; // no fifo
    comSettingsNew.rx_timeout        = RTSER_TIMEOUT_NONE;//0000000; //RTSER_TIMEOUT_NONE;//;20000000;//RTSER_TIMEOUT_NONE;
    comSettingsNew.tx_timeout        = RTSER_TIMEOUT_NONE;
    comSettingsNew.event_timeout     = RTSER_TIMEOUT_NONE;
    comSettingsNew.timestamp_history = RTSER_RX_TIMESTAMP_HISTORY;
    comSettingsNew.event_mask        = RTSER_EVENT_RXPEND;

    rt_dev_ioctl(_hCom, RTSER_RTIOC_SET_CONFIG, comSettingsNew);

    nRetval = 1;
  }
	else nRetval = 0;

#else

	COMMSETTINGS comSettingsNew;
	
	_hCom = open(_szDeviceName, O_RDWR | O_NOCTTY); 

	//printf("%s %d ", _szDeviceName, _hCom);
	
	if (_hCom >= 0)
	{
		tcgetattr(_hCom,&_comSettings); /* save current port settings */
		/*
		 * this is deprecated
		 * bzero(&comSettingsNew,sizeof(comSettingsNew));
		 * using instead:
		 */
		memset(&comSettingsNew, 0, sizeof(comSettingsNew));

		comSettingsNew.c_cflag = (int)_eBaudrate | CS8 | CLOCAL | CREAD;
		/* set input mode raw */
		comSettingsNew.c_lflag = 0;
		comSettingsNew.c_iflag = 0;
		comSettingsNew.c_oflag = 0;
		comSettingsNew.c_cc[VTIME]    = 0;   /* inter-character timer unused */
		comSettingsNew.c_cc[VMIN]     = 0;   /* nonblocking read */

		tcflush(_hCom, TCIFLUSH);
		tcsetattr(_hCom,TCSANOW,&comSettingsNew);
		nRetval = 1;
	}
	else nRetval = 0;

#endif

	return nRetval;
}

int CComAdapter::closeDevice()
{
	int nRetval = 0;
	if(_hCom >= 0){
		#ifdef WIN32
			if(CloseHandle(_hCom)) nRetval = 1;
		#elif REAL_TIME
			if(rt_dev_close(_hCom)) nRetval = 1;
		#else
			if(close(_hCom)) nRetval = 1;
		#endif
	}
	_hCom = -1;
	return nRetval;
}

unsigned long CComAdapter::send(char* szMessage, unsigned long ulLength)
{
	if (_hCom < 0) return 0;
	
	unsigned long ulBytesWritten;
	#ifdef WIN32	
		WriteFile( _hCom, szMessage, ulLength, &ulBytesWritten, NULL);
	#elif REAL_TIME
		ulBytesWritten = rt_dev_write(_hCom, szMessage, ulLength);
	#else
		ulBytesWritten = write(_hCom, szMessage, ulLength);
	#endif
	return ulBytesWritten;
}

unsigned long CComAdapter::receive(char* szAnswer, unsigned long ulLength)
{
	return (this->*_fptrReceive)(szAnswer, ulLength);	
}

unsigned long CComAdapter::receiveUnbuffered(char* szAnswer, unsigned long ulLength)
{
	if (_hCom < 0) return 0;
	
	unsigned long ulBytesRead;
	#ifdef WIN32
		ReadFile ( _hCom, szAnswer, ulLength, &ulBytesRead, NULL);
	#elif REAL_TIME
		//ulBytesRead = rt_dev_read(_hCom,szAnswer,ulLength);
//		int ulBR = rt_dev_read(_hCom,szAnswer,ulLength);
		int ulBR = rt_dev_read(_hCom,szAnswer,1000);
    //////////////////// READ TIMESTAMP OF LAST BYTE
    uint64_t  ts;
    rt_dev_ioctl(_hCom,123456, &ts);
    setTimeStamp(ts);
//    printf("B %d     %d\n", ulBR, ulLength);
    if (ulBR < 0){
      ulBytesRead = 0;
//      printf("ERROR %d %s\n", ulBR, strerror(-ulBR));
    }
    else {
      ulBytesRead = ulBR;
/*      for(int i = 0; i<ulBR;i++) {
        printf("%d ", szAnswer[i]);
      }
      printf("\n");*/
    }
	#else
		ulBytesRead = read(_hCom,szAnswer,ulLength);
	#endif
	//printf("Bytes read: %d\n",ulBytesRead);
	return ulBytesRead;
}

unsigned long CComAdapter::receiveBuffered(char* szAnswer, unsigned long ulLength)
{
	if (_hCom < 0) return 0;
	
	unsigned long ulBytesRead;
	ulBytesRead = 0;
		
	// Buffer message
	unsigned long ulBytesToBeRead = ulLength-_ulBytesPrevRead;

	unsigned long ulBytesReadBuf = 0;
	#ifdef WIN32
		ReadFile ( _hCom, &_szBuffer[_ulBytesPrevRead], ulBytesToBeRead, &ulBytesReadBuf, NULL);
	#elif REAL_TIME
		ulBytesReadBuf =	rt_dev_read(_hCom,&_szBuffer[_ulBytesPrevRead],ulBytesToBeRead);
	#else
		ulBytesReadBuf =	read(_hCom,&_szBuffer[_ulBytesPrevRead],ulBytesToBeRead);
	#endif

	if(ulBytesReadBuf==((unsigned short)ulLength - _ulBytesPrevRead))
	{
		ulBytesRead = (unsigned short)ulLength;
		memcpy(szAnswer,_szBuffer,ulLength);
		_ulBytesPrevRead = 0;
	}
	else
	{
		_ulBytesPrevRead += ulBytesReadBuf;
	}

	return ulBytesRead;
}

int CComAdapter::clearReceiveBuffer()
{
	if (_hCom < 0) return 0;
	
	#ifdef WIN32
		PurgeComm(_hCom, PURGE_RXCLEAR); 
	#elif REAL_TIME
	#else
		tcflush(_hCom, TCIFLUSH);
	#endif
	return 0;
}

int CComAdapter::clearTransmissionBuffer()
{
	if (_hCom < 0) return 0;
	
	#ifdef WIN32
		PurgeComm(_hCom, PURGE_TXCLEAR);
	#elif REAL_TIME
	#else
		tcflush(_hCom, TCOFLUSH);
	#endif
	return 0;
}

}
