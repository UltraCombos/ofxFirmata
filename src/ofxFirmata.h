/*
 * Copyright 2007-2008 (c) Erik Sjodin, eriksjodin.net
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
#pragma once

#include <list>
#include "ofConstants.h"

#include "ofEvents.h"

#include "ofSerial.h"

/*
 * Version numbers for the protocol. The protocol is still changing, so these
 * version numbers are important. This number can be queried so that host
 * software can test whether it will be compatible with the currently installed firmware.
 */

#define FIRMATA_MAJOR_VERSION   2 // for non-compatible changes
#define FIRMATA_MINOR_VERSION   0 // for backwards compatible changes
#define FIRMATA_MAX_DATA_BYTES 32 // max number of data bytes in non-Sysex messages

#undef ANALOG_MAPPING_QUERY
#undef ANALOG_MAPPING_RESPONSE
#undef CAPABILITY_QUERY
#undef CAPABILITY_RESPONSE
#undef PIN_STATE_QUERY        
#undef PIN_STATE_RESPONSE     
#undef EXTENDED_ANALOG
#undef SHIFT_DATA
#undef I2C_REQUEST
#undef I2C_REPLY
#undef I2C_CONFIG
#undef SAMPLING_INTERVAL

// message command bytes (128-255/0x80-0xFF)
enum class MessageType
{
	// extended command set using SysEx (0-127/0x00-0x7F)
	/* 0x00-0x0F reserved for custom commands */
	ANALOG_MAPPING_QUERY    = 0x69, // ask for mapping of analog to pin numbers
	ANALOG_MAPPING_RESPONSE = 0x6A, // reply with mapping info
	CAPABILITY_QUERY        = 0x6B, // ask for supported modes and resolution of all pins
	CAPABILITY_RESPONSE     = 0x6C, // reply with supported modes and resolution
	PIN_STATE_QUERY         = 0x6D, // ask for a pin's current mode and value
	PIN_STATE_RESPONSE      = 0x6E, // reply with pin's current mode and value
	EXTENDED_ANALOG         = 0x6F, // analog write (PWM, Servo, etc) to any pin
	//
	SERVO_CONFIG            = 0x70, // set max angle, minPulse, maxPulse, freq
	STRING_DATA             = 0x71, // a string message with 14-bits per char
	//
	SHIFT_DATA              = 0x75, // a bitstram to/from a shift register
	I2C_REQUEST             = 0x76, // send an I2C read/write request
	I2C_REPLY               = 0x77, // a reply to an I2C request
	I2C_CONFIG              = 0x78, // config I2C settings such as delay times and power pins
	REPORT_FIRMWARE         = 0x79, // report name and version of the firmware
	//
	SAMPLING_INTERVAL       = 0x7A, // set the poll rate of the main loop
	//
	//SYSEX_NON_REALTIME    = 0x7E, // MIDI Reserved for non-realtime messages
	//SYSEX_REALTIME        = 0x7F, // MIDI Reserved for realtime messages
	//
	ANALOG_IO_MESSAGE       = 0xE0, // send data for an analog pin (or PWM) use EXTENDED_ANALOG instead
	DIGITAL_IO_MESSAGE      = 0x90, // send data for a digital pin
	REPORT_ANALOG_PIN       = 0xC0, // enable analog input by pin #
	REPORT_DIGITAL_PORT     = 0xD0, // enable digital input by port pair
	//				        
	START_SYSEX             = 0xF0, // start a MIDI Sysex message
	//					    
	SET_PIN_MODE            = 0xF4, // set a pin to INPUT/OUTPUT/PWM/etc
	SET_DIGITAL_PIN_VALUE   = 0xF5, // set a pin to INPUT/OUTPUT/PWM/etc
	//					    
	END_SYSEX               = 0xF7, // end a MIDI Sysex message
	PROTOCOL_VERSION        = 0xF9, // report protocol version 	major version	minor version
	//					    
	SYSTEM_RESET            = 0xFF, // reset from MIDI
};

#undef SHIFT
#undef I2C

// pin modes
enum class PinMode
{
	_NULL          = -1,
	DIGITAL_INPUT  = 0x00,
	DIGITAL_OUTPUT = 0x01,
	ANALOG_INPUT   = 0x02,
	PWM            = 0x03,
	SERVO          = 0x04,
	SHIFT          = 0x05,
	I2C            = 0x06,
	ONEWIRE        = 0x07,
	STEPPER        = 0x08,
	ENCODER        = 0x09,
	SERIAL         = 0x0A,
	INPUT_PULLUP   = 0x0B,
};

enum class DigitalValue
{
	LOW = 0,
	HIGH = 1
};

// ---- arduino constants (for Arduino NG and Diecimila)

// board settings
#define ARD_TOTAL_PORTS         3 // total number of ports for the board

#define OF_ARDUINO_DELAY_LENGTH 4.0

#define FIRMWARE2_2             22
#define FIRMWARE2_3             23


/// \brief This is a way to control an Arduino that has had the firmata library
/// loaded onto it, from OF.
///
/// To load firmata onto your Arduino, run the Arduino IDE, open the Examples >
/// Firmata > StandardFirmata sketch, and upload it to the Arduino board.
///
/// Once the ofArduino instance returns true from isArduinoReady() you can set
/// the mode of the different digital pins using sendDigitalPinMode()
///
/// This sets pin 9 to input so that it can read a button press
/// ~~~~{.cpp}
///     sendDigitalPinMode(9, ARD_INPUT)
/// ~~~~
///
/// This sets pin 9 to be a PWM out pin. Note that this only works on pins
/// that are PWM enabled.
/// ~~~~{.cpp}
///     sendDigitalPinMode(9, ARD_PWM)
/// ~~~~
class ofxFirmata {

	public:
		/// \name Constructor and Destructor
		/// \{
		ofxFirmata();

		virtual ~ofxFirmata();

		/// \}
		/// \name Connect
		/// \{

		/// \brief Opens a serial port connection to the arduino
		/// \param device The name of the device.
		/// You can get the name from the Arduino IDE
		/// \param baud The baud rate the connection uses
		bool connect(const std::string & device, int baud = 57600);

		/// \brief Returns true if a succesfull connection has been established
		/// and the Arduino has reported a firmware
		bool isInitialized() const;

		bool isArduinoReady() const;

		/// \brief Closes the serial port connection.
		/// Does not turn the Arduino off.
		void disconnect();

		/// \}
		/// \name Update
		/// \{

		/// \brief Polls data from the serial port, this has to be called periodically
		void update();

		/// \}
		/// \name Setup
		/// \{

		/// \brief Setting a pins mode to ARD_INPUT turns on reporting for the port the pin is on
		/// \param pin Pin on arduino (2-13)
		/// \param mode `ARD_INPUT`, `ARD_OUTPUT`, `ARD_PWM`
		/// \note Analog pins 0-5 can be used as digitial pins 16-21 but if the
		/// mode of _one_ of these pins is set to `ARD_INPUT` then _all_ analog pin
		/// reporting will be turned off
		void sendDigitalPinMode(int pin, PinMode mode);

		void sendAnalogPinReporting(int pin, bool reporting);
		// pin: 0-5
		// mode: ARD_ON or ARD_OFF
		// Note: analog pins 0-5 can be used as digitial pins 16-21 but if reporting for _one_ analog pin is enabled then reporting for _all_ of digital pin 16-21 will be turned off

		void setUseDelay(bool bDelay);

		void setDigitalHistoryLength(int length);
		void setAnalogHistoryLength(int length);
		void setStringHistoryLength(int length);
		void setSysExHistoryLength(int nSysEx);

		/// \}
		/// \name Senders
		/// \{

		void sendDigital(int pin, DigitalValue value, bool force = false);
		// pin: 2-13
		// value: ARD_LOW or ARD_HIGH
		// the pins mode has to be set to ARD_OUTPUT or ARD_INPUT (in the latter mode pull-up resistors are enabled/disabled)
		// Note: pin 16-21 can also be used if analog inputs 0-5 are used as digital pins

		void sendPwm(int pin, int value, bool force = false);

		void sendSysEx(MessageType command, vector <unsigned char> data);
		void sendSysEx(MessageType command)
		{
			sendByte(MessageType::START_SYSEX);
			sendByte(command);
			sendByte(MessageType::END_SYSEX);
		}

		/// \brief Send a string to the Arduino
		/// \note Firmata can not handle strings longer than 12 characters.
		void sendString(string str);

		void sendProtocolVersionRequest();

protected:
		void sendFirmwareVersionRequest();
public:
		/// \brief This will cause your Arduino to reset and boot into the program again.
		void sendReset();

		/// \brief Sends the `FIRMATA_START_SYSEX` command
		void sendSysExBegin();

		/// \brief Sends the `FIRMATA_END_SYSEX` command
		void sendSysExEnd();

		/// \brief Sends a byte without wrapping it in a firmata message.
		///
		/// Data has to be in the 0-127 range. Values > 127 will be interpreted as
		/// commands.
		void sendByte(unsigned char byte);

		/// \brief Send value as two 7 bit bytes.
		///
		/// Sends a value as two 7-bit bytes without wrapping it in a firmata
		/// message.  Values in the range 0 - 16384 will be sent as two bytes
		/// within the 0-127 data range.
		///
		/// \param value The value to send.
		void sendValueAsTwo7bitBytes(int value);

		/// \}
		/// \name Getters
		/// \{

		/// \brief Returns the last set PWM value (0-255) for the given pin
		///
		/// The pins mode has to be ARD_PWM
		///
		/// On the Arduino Uno the following pins are supported: 3, 5, 6, 9, 10 and 11
		/// \note Pin 16-21 can also be used if analog inputs 0-5 are used as digital pins
		int getPwm(int pin) const;

		/// \brief Returns the last received value (if the pin mode is ARD_INPUT)
		/// or the last set value (if the pin mode is ARD_OUTPUT) for the given
		/// pin
		///
		/// Returns whether the pin is reading high or low, 1 or 0. You can test
		/// against this with an if() statement which is handy:
		/// ~~~~{.cpp}
		///     if(arduino.getDigital(pin)){
		///         // do something on high
		///     } else {
		///         // do something on low
		///     }
		/// ~~~~
		/// \note Pin 16-21 can also be used if analog inputs 0-5 are used as digital pins
		int getDigital(int pin) const;

		/// \brief Returns the analog in value that the pin is currently reading.
		/// because the Arduino has a 10 bit ADC you get between 0 and 1023 for
		/// possible values.
		///
		/// \param pin The pin number (0-5)
		int getAnalog(int pin) const;

		/// \returns the last received SysEx message.
		vector <unsigned char> getSysEx() const;

		/// \returns the last received string.
		string getString() const;

		/// \brief Returns the major firmware version
		int getMajorProtocolVersion() const;

		/// \returns the minor firmware version.
		int getMinorProtocolVersion() const;

		/// \returns the major firmware version.
		int getMajorFirmwareVersion() const;

		/// \returns the minor firmware version.
		int getMinorFirmwareVersion() const;

		/// \returns the name of the firmware.
		string getFirmwareName() const;

		/// \brief Returns a pointer to the digital data history list for the
		/// given pin
		/// \note Pin 16-21 can also be used if analog inputs 0-5 are used as
		/// digital pins
		/// \param pin The pin number (2-13)
		list <int> * getDigitalHistory(int pin);

		/// \brief Returns a pointer to the analog data history list for the given pin.
		/// \param pin The Arduino Uno pin: 0-5
		list <int> * getAnalogHistory(int pin);

		/// \returns a pointer to the SysEx history.
		list <vector <unsigned char> > * getSysExHistory();

		/// \returns a pointer to the string history.
		list <string> * getStringHistory();

		/// \brief Get the pin mode of the given pin
		///
		/// \returns `ARD_INPUT`, `ARD_OUTPUT`, `ARD_PWM`, `ARD_SERVO`, `ARD_ANALOG`
		PinMode getDigitalPinMode(int pin) const;

		/// \returns `ARD_ON` or `ARD_OFF`
		bool getAnalogPinReporting(int pin) const;

		/// \brief Useful for parsing SysEx messages
		int getValueFromTwo7bitBytes(unsigned char lsb, unsigned char msb);

		/// \}
		/// \name Events
		/// \{

		/// \brief Triggered when a digital pin changes value, the pin that
		/// changed is passed as an argument.
		ofEvent <const int> EDigitalPinChanged;

		/// \brief Triggered when an analog pin changes value, the pin that
		/// changed is passed as an argument.
		ofEvent <const int> EAnalogPinChanged;

		/// \brief Triggered when a SysEx message that isn't in the extended
		/// command set is received, the SysEx message is passed as an argument
		ofEvent <const vector <unsigned char> > ESysExReceived;

		/// \brief Triggered when a protocol version is received, the major version
		/// is passed as an argument.
		ofEvent <const int> EProtocolVersionReceived;

		/// \brief Triggered when a firmware version is received, the major version
		/// is passed as an argument.
		ofEvent <const int> EFirmwareVersionReceived;

		/// \brief Triggered when the firmware version is received upon connect,
		/// the major firmware version is passed as an argument. From this point
		/// it's safe to send to the Arduino.
		ofEvent <const int> EInitialized;

		/// \brief Triggered when a string is received, the string is passed as an
		/// argument
		ofEvent <const string> EStringReceived;

		/// \}
		/// \name Servos
		/// \{

		/// \brief Send a value to a servo.
		///
		/// A servo has to be atached to the pin prior
		/// \param pin 9 or 10
		/// \param value The value to send
		void sendServo(int pin, int value, bool force = false);

		/// \param angle parameter DEPRECATED as of Firmata 2.2
		void sendServoConfig(int pin, int minPulse = 544, int maxPulse = 2400);

		/// \returns the last set servo value for a pin if the pin has a servo attached.
		int getServo(int pin) const;

		/// \}

	protected:
		mutable bool _initialized = false; ///\< \brief Indicate that pins are initialized.

		void initPins() const;

		void sendDigitalPinReporting(int pin, bool reporting);
		// sets pin reporting to ARD_ON or ARD_OFF
		// enables / disables reporting for the pins port

		void sendDigitalPortReporting(int port, bool mode);
		// sets port reporting to ARD_ON or ARD_OFF
		// enables / disables reporting for ports 0-2
		// port 0: pins 2-7  (0,1 are serial RX/TX)
		// port 1: pins 8-13 (14,15 are disabled for the crystal)
		// port 2: pins 16-21 analog pins used as digital, all analog reporting will be turned off if this is set to ARD_ON

		void processData(unsigned char inputData);
		void processDigitalPort(int port, unsigned char value);
		virtual void processSysExData(vector <unsigned char> data);

		ofSerial _port;
		int _portStatus = -1;

		// --- history variables
		int _analogHistoryLength = 2;
		int _digitalHistoryLength = 2;
		int _stringHistoryLength = 1;
		int _sysExHistoryLength = 1;

		// --- data processing variables
		int _waitForData = 0;
		int _executeMultiByteCommand; ///< \brief Indicate Firmata command to execute.
		int _multiByteChannel = 0; ///< \brief Indicates which pin the data came from.

		// --- data holders
		unsigned char _storedInputData[FIRMATA_MAX_DATA_BYTES];
		vector <unsigned char> _sysExData;
		int _majorProtocolVersion = 0;
		int _minorProtocolVersion = 0;
		int _majorFirmwareVersion = 0;
		int _minorFirmwareVersion = 0;
		string _firmwareName = "Unknown";;

		// sum of majorFirmwareVersion * 10 + minorFirmwareVersion -> Firmata (?)
		int _firmwareVersionSum = 0;

		list <vector <unsigned char> > _sysExHistory;
		// maintains a history of received sysEx messages (excluding SysEx messages in the extended command set)

		list <string> _stringHistory;
		// maintains a history of received strings

		using PinResolution = int;
		using PinCapability = map<PinMode, PinResolution>;
		vector<PinCapability> _pin_capabilites;

		struct AnalogPin
		{
			int digitalPinNum = -1;
			int value = -1;
			bool reporting = false;
			list <int> history;
		};

		struct DigitalPin
		{
			PinMode mode = PinMode::_NULL;
			int value = -1;
			bool reporting = false;
			list <int> history;
		};

		vector<AnalogPin> _analog_pins;
		mutable vector<DigitalPin> _digital_pins;
		
		mutable vector<int> _digitalPortValue;
		// the last set values on all ports

		mutable vector<bool> _digitalPortReporting;
		// whether pin reporting is enabled / disabled

		bool bUseDelay = true;

		mutable bool connected = false; ///< \brief This yields true if a serial connection to Arduino exists.

		float connectTime = 0.0f; ///< \brief This represents the (running) time of establishing a serial connection.

		// the last set servo values
		void sendByte(MessageType msg)
		{
			sendByte((int)msg);
		}
		void sendCapabilityQuery()
		{
			sendSysEx(MessageType::CAPABILITY_QUERY);
		}
		void sendAnalogMappingQuery()
		{
			sendSysEx(MessageType::ANALOG_MAPPING_QUERY);
		}
		void sendExtendedAnalog(int pin, int value)
		{
			sendSysExBegin();
			sendByte(MessageType::EXTENDED_ANALOG);
			sendByte(pin);
			sendValueAsTwo7bitBytes(value);
			sendSysExEnd();
		}
		void sendPinStateQuery(int pin)
		{
			sendSysExBegin();
			sendByte(MessageType::PIN_STATE_QUERY);
			sendByte(pin);
			sendSysExEnd();
		}
		void tryInit()
		{
			if(_minorFirmwareVersion == 0 && _majorFirmwareVersion == 0)
				sendFirmwareVersionRequest();

			if(_pin_capabilites.size() == 0)
				sendCapabilityQuery();

			if(_analog_pins.size() == 0)
				sendAnalogMappingQuery();

			if (_minorFirmwareVersion != 0 && 
				_majorFirmwareVersion != 0 && 
				_pin_capabilites.size() != 0 && 
				_analog_pins.size() != 0)
			{
				for (int i = 0; i < _pin_capabilites.size(); i++)
				{
					sendPinStateQuery(i);
				}
				initPins();
				ofNotifyEvent(EInitialized, _majorFirmwareVersion, this);
			}
		}
		string PinModeToString(PinMode mode)
		{
			switch (mode)
			{
			case PinMode::DIGITAL_INPUT:
				return "DIGITAL_INPUT";
				break;
			case PinMode::DIGITAL_OUTPUT:
				return "DIGITAL_OUTPUT";
				break;
			case PinMode::ANALOG_INPUT:
				return "ANALOG_INPUT";
				break;
			case PinMode::PWM:
				return "PWM";
				break;
			case PinMode::SERVO:
				return "SERVO";
				break;
			case PinMode::SHIFT:
				return "SHIFT";
				break;
			case PinMode::I2C:
				return "I2C";
				break;
			case PinMode::ONEWIRE:
				return "ONEWIRE";
				break;
			case PinMode::STEPPER:
				return "STEPPER";
				break;
			case PinMode::ENCODER:
				return "ENCODER";
				break;
			case PinMode::SERIAL:
				return "SERIAL";
				break;
			case PinMode::INPUT_PULLUP:
				return "INPUT_PULLUP";
				break;
			default:
				return "_NULL";
				break;
			}
		}
};

//typedef ofArduino ofStandardFirmata;

