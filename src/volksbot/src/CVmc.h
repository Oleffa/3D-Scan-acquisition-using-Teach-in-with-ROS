
#ifdef WIN32
#include <windows.h>
#else
#include <pthread.h>
#endif


namespace VMC {

/**
 * @class CVmc
 * @brief Class CVmc implements VMC motor control functions.
 *
 *		This class can be used for accessing the Fraunhofer
 *		Volksbot Motor Controller (VMC). It provides a high-level
 *		application programming interface (API).
 *		
 *		Motors connected to the outputs can be controlled by the 
 *		'setMotor...(...)' commands. Sensor values which refer to the state
 *		of the motors can be read by the 'getMotorValue...(...)' commands.
 *		Other state values can be read by the 'getValue(...)' command.
 *		
 *		The available types of state values are defined as 'static const int'
 *		attributes of this class. These attributes are used as parameters for
 *		many functions of this API. Their meaning is described in the
 *		'Static Public	Attributes' section above.
 *
 *		All messaging actions between PC and VMC are encapsulated so that the
 *		developer does not have to build a messaging loop and does not
 *		have to deal with low level protocol details. If you use this class
 *		there is a seperate thread running in the background, communicating 
 *		with the VMC 50 times per second (20ms cycle time). In every cycle
 *		the motors are set and state values are returned by the VMC.
 *
 *		NOTE: There is only one type of state value returned per cycle. 
 *		For reading more than one type there is a 'response list' containing 
 *		the types of state values which should be refreshed cyclical
 *		while the application is running. One type of state values  
 *		is read out of the VMC per cycle. When the list reached its end
 *		it will start from the beginning.
 *
 *		This means when there is only one type of state value in the list
 *		it will be updated every cycle, if there are two types in the list
 *		they are updated every second cycle, and so on...
 *		
 *		The default behaviour of this class is to have MOTOR_TICKS_ABSOLUTE
 *		in the response list, so that these values (for all three motors) are
 *		updated in every cycle.
 *
 *		Simple example: If you add another state value type 
 *		(e.g. the battery voltage) by calling 
 *		addValueToResponseList(BATTERY_VOLTAGE) there are two state value
 *		types in the list and they are updated every second cycle. 
 *
 *		
 * @version 0.95z
 * @author Bjoern Flintrop
 */

class CVmc
{

public:

	/*
	 * These static constants describe the state values which can be read
	 * out of the motor controller.
	 */
	static const int BATTERY_VOLTAGE; /**< @brief actual battery voltage (mV) */
	static const int MOTOR_CURRENT; /**< @brief actual current of the motor (mA)*/
//	static const int MOTOR_POWER; /**< @brief actual motor power */
	static const int MOTOR_PWM; /**< @brief value of pulse width modulation */
  	static const int MOTOR_RPM; /**< @brief actual rotations per minute */
//	static const int MOTOR_TEMPERATURE; /**< @brief actual motor temperature */
	static const int MOTOR_TICKS_ABSOLUTE; /**< @brief absolute ticks based on encoder input */
	static const int MOTOR_TICKS_RELATIVE; /**< @brief tick count since last internal cycle */

	static const int VMC_ERROR; /**< @brief this value is returned if a 'get...' function fails */

	/**
	 * The default constructor opens a connection to the VMC.
	 * The COM port is detected automatically in a range from COM1 to COM9. 
	 * You can use the other constructor if this leads to problems.
	 */
	CVmc();

	/**
	 * This constructor opens a connection to the VMC.
	 * The specified COM port is used.
	 *
	 * @param comPort the COM Port name (e.g. "COM1", "/dev/ttyS1" etc.)
	 */
	CVmc(const char* comPort);

	/**
  	 * The default destructor closes the connection to VMC
  	 */
	~CVmc();

	/**
	 * Add a state value to the response list.
	 * It will be updated every N-th cycle, with N being the
	 * number of elements in the list.
	 * @param state the type of state value to be added
	 *		(see static public attributes for state value constants)
	 */
	void addStateToResponseList(int state);

	/**
	 * Clear the response list so that there is no state
	 * value updated any longer.
	 */
	void clearResponseList();

	/**
	 * Get the actual COM port which the VMC is connected to.
	 * @return the COM Port string (e.g. "COM1", "/dev/ttyS0" etc.)
	 *    if there is a connection, otherwise returns "No connection!".
	 */
	char* getComPortString();

	/**
	 * Get the value of a digital input.
	 * @param input specifies the input from which the value is returned (1-3)
	 * @return 1 if the input is at +5V, 0 if the input is at 0V, and
	 *		VMC_ERROR if the port is not set as input (You have to do
	 *    this by explicitly calling 'setDigitalIoConfiguration' at the
	 *		beginning of your program).
	 */
	int getDigitalValue(int input);

	/**
	 * Get a motor state value.
	 * @param motor specifies the motor from which the value is returned (1-3)
	 * @param state specifies the type of the state value to be returned
	 *		(see static public attributes for state value constants)
	 * @return the specified motor status value if it is in the response list,
	 *		otherwise VMC_ERROR is returned.
	 */
	int getMotorValue(int motor, int state);

	/**
	 * Get a motor state value from the left motor
	 * which is assumed to be connected to output 2.
	 * The value is being inverted (except in case of MOTOR_CURRENT which
	 * is always a positive value) for getting positive values while driving
	 * forward. (which the right motor input gets without inverting).
	 * @param state specifies the type of the state value to be returned 
	 *		(see static public attributes for state value constants)
	 * @return the specified motor status value if its in the response list,
	 *		otherwise VMC_ERROR is returned.
	 */
	int getMotorValueLeft(int state);

	/**
	 * Get a motor motor state value from the right motor
	 * which is assumed to be connected to output 1.
	 * @param state specifies the type of the state value to be returned 
	 *		(see static public attributes for status value constants)
	 * @return the specified motor status value if its in the response list,
	 *		otherwise VMC_ERROR is returned.
	 */
	int getMotorValueRight(int state);

	/**
	 * Get a state value.
	 * @param state specifies the type of the state value to be returned
	 *		(see static public attributes for state value constants)
	 * @return the specified state value if its in the response list,
	 *		otherwise VMC_ERROR is returned.
	 */
	int getValue(int state);

	/**
	 * Returns if there is a connection to the VMC established.
	 *
	 * @return true if there is a connection, otherwise false
	 */
	bool isConnected();

	/**
	 * Remove a state value from the response list.
	 * It will be not updated any longer.
	 * @param state the not longer required type of state value
	 *		(see static public attributes for state value constants)
	 */
	void removeStateFromResponseList(int state);

	/**
	 * The value of MOTOR_TICKS_ABSOLUTE is set to zero for all
	 * three motor encoder input ports.
	 */
	void resetMotorTicks();

	/**
	 * The value of MOTOR_TICKS_ABSOLUTE is set to zero for
	 * one motor encoder input port.
	 *
	 * @param motor specifies the motor encoder input 
	 *		which should be set to zero (1-3).
	 */
	void resetMotorTicks(int motor);

	/**
	 * Set the cycle time for the RS-232 messaging loop.
	 * The default value is 20. Valid values are in the range
	 * from 20 to 1000. Note that the VMC has a timeout setting.
	 *
	 * @param milliseconds the duration of one cycle in milliseconds
	 */
	void setCycleTime(int milliseconds);

	/**
	 * Set the IO configuration for the three digital in-/output ports and
	 * enable digital input/output processing.
	 * Setting an IO port to true configures it as an output port, setting
	 * it to false configures it as an input port.
	 * @param io1 specifies if port 1 is input (false) or output (true)
	 * @param io2 specifies if port 2 is input (false) or output (true)
	 * @param io3 specifies if port 3 is input (false) or output (true)
	 */
	void setDigitalIOConfiguration(bool io1, bool io2, bool io3);

	/**
	 * Set the digital value for a specific output.
	 * @param output specifies the output to be set (1-3).
	 * @param value specifies: +5V if true, 0V if false.
	 */
	void setDigitalValue(int output, bool value);

	/**
	 * Set the digital values for all three outputs.
	 * @param output1 +5V if true, 0V if false
	 * @param output2 +5V if true, 0V if false
	 * @param output3 +5V if true, 0V if false
	 */
	void setDigitalValues(bool output1, bool output2, bool output3);

	/**
	 * Set the maximum rotations per minute for the motors.
	 * The default value is 6000. Valid values are in the range
	 * from 0 to 8000.
	 *
	 * @param rpm the maximum rotations per minute (0-8000)
	 */
	void setMaximumRPM(int rpm);

	/**
	 * Set the desired velocity for the left motor
	 * which is assumed to be connected to output 2.
	 * The value is being inverted for letting the left wheel drive
	 * forward when getting positive values (which the right wheel does
	 * without inverting).
	 * @param velocity the desired velocity (-100 - +100)
	 */
	void setMotorLeft(int velocity);

	/**
	 * Set the desired velocity for the right motor.
	 * which is assumed to be connected to output 1.
	 * @param velocity the desired velocity (-100 - +100)
	 */
	void setMotorRight(int velocity);

	/**
	 * Set the desired rotations per minute for a motor.
	 * @param motor specifies the motor (1-3).
	 * @param rpm the desired rotations per minute (-8000 - +8000)
	 */
	void setMotorRPM(int motor, int rpm);

	/**
	 * Set the desired velocity for a motor.
	 * @param motor specifies the motor (1-3).
	 * @param velocity the desired velocity (-100 - +100)
	 */
	void setMotorVelocity(int motor, int velocity);

	/**
	 * Set the desired velocity for all motors.
	 * @param velocity1 the desired velocity for motor at output 1 (-100 - +100)
	 * @param velocity2 the desired velocity for motor at output 2 (-100 - +100)
	 * @param velocity3 the desired velocity for motor at output 3 (-100 - +100)
	 */
	void setMotors(int velocity1, int velocity2, int velocity3);

	/**
	 * Set the desired velocity for the motors of the drive.
	 * The left value is being inverted for letting the left wheel drive
	 * forward when getting positive values (which the right wheel does
	 * without inverting).
	 * @param velocityLeft the desired velocity for left motor 
	 *		at output 2 (-100 - +100)
	 * @param velocityRight the desired velocity for right motor 
	 *		at output 1 (-100 - +100)
	 */
	void setMotors(int velocityLeft, int velocityRight);

	/**
	 * Wait for a period of time.
	 * The processor is not used while waiting.
	 * @param milliseconds the milliseconds to wait for
	 */
	void wait(int milliseconds);

private:

	bool _isConnected;
	char _comPort[41];
	bool _responseList[10];
	
	int _pwmOut1, _pwmOut2, _pwmOut3;
	bool _digitalIO1, _digitalIO2, _digitalIO3;
	bool _digitalOut1, _digitalOut2, _digitalOut3;
	bool _digitalInputUpdate;
	int _maxRpm;
	int _cycleTime;

#ifdef WIN32
	CRITICAL_SECTION _cs;
   friend DWORD WINAPI vmcThreadFunction(LPVOID param);
#else
	pthread_mutex_t _mutex;
   friend void* vmcThreadFunction(void* param);
#endif

	void enterCriticalSection();
	void leaveCriticalSection();
	void init(const char* comPort);
	int *_apiObject;
};


} // namespace VMC
