/*
 * updated 2017 by chwan1@ultracombos.com 
 *
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
	DIGITAL_IO_MESSAGE      = 0x90, // send data for a digital pin
	REPORT_ANALOG_PIN       = 0xC0, // enable analog input by pin #
	REPORT_DIGITAL_PORT     = 0xD0, // enable digital input by port pair
	ANALOG_IO_MESSAGE       = 0xE0, // send data for an analog pin (or PWM)
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

#if 0
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

#else
#define VALUE_TABLE \
 X(_NULL          , = -1   , "_NULL"          ) \
 X(DIGITAL_INPUT  , = 0x00 , "DIGITAL_INPUT"  ) \
 X(DIGITAL_OUTPUT , = 0x01 , "DIGITAL_OUTPUT" ) \
 X(ANALOG_INPUT   , = 0x02 , "ANALOG_INPUT"   ) \
 X(PWM            , = 0x03 , "PWM"            ) \
 X(SERVO          , = 0x04 , "SERVO"          ) \
 X(SHIFT          , = 0x05 , "SHIFT"          ) \
 X(I2C            , = 0x06 , "I2C"            ) \
 X(ONEWIRE        , = 0x07 , "ONEWIRE"        ) \
 X(STEPPER        , = 0x08 , "STEPPER"        ) \
 X(ENCODER        , = 0x09 , "ENCODER"        ) \
 X(SERIAL         , = 0x0A , "SERIAL"         ) \
 X(INPUT_PULLUP   , = 0x0B , "INPUT_PULLUP"   ) \

#define X(a, b, c) a b,
enum PinMode { VALUE_TABLE };
#undef X
extern const string ToString(PinMode value);
extern const PinMode FromString(string value);
#endif

enum class DigitalValue
{
	LOW = 0,
	HIGH = 1
};

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
		enum class ReportStrategy
		{
			None,
			Always,
			OnChange,
		};

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

		// pin: 0-5
		// mode: ARD_ON or ARD_OFF
		// Note: analog pins 0-5 can be used as digitial pins 16-21 but if reporting for _one_ analog pin is enabled then reporting for _all_ of digital pin 16-21 will be turned off

		void setUseDelay(bool bDelay);

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

		void sendDigitalPinReporting(int pin, ReportStrategy reportStrategy);
		/// \brief Setting a pins mode to ARD_INPUT turns on reporting for the port the pin is on
		/// \param pin Pin on arduino (2-13)
		/// \param mode `ARD_INPUT`, `ARD_OUTPUT`, `ARD_PWM`
		/// \note Analog pins 0-5 can be used as digitial pins 16-21 but if the
		/// mode of _one_ of these pins is set to `ARD_INPUT` then _all_ analog pin
		/// reporting will be turned off
		void sendDigitalPinMode(int pin, PinMode mode, bool force = false);

		/// \brief Get the pin mode of the given pin
		///
		/// \returns `ARD_INPUT`, `ARD_OUTPUT`, `ARD_PWM`, `ARD_SERVO`, `ARD_ANALOG`
		PinMode getDigitalPinMode(int pin) const;

		void sendAnalogPinReporting(int pin, ReportStrategy reportStrategy, bool force = false);
		/// \returns `ARD_ON` or `ARD_OFF`
		ReportStrategy getAnalogPinReporting(int pin) const;

		/// \brief Returns the analog in value that the pin is currently reading.
		/// because the Arduino has a 10 bit ADC you get between 0 and 1023 for
		/// possible values.
		///
		/// \param pin The pin number (0-5)
		int getAnalog(int pin) const;

		/// \}
		/// \name Senders
		/// \{

		void sendDigital(int pin, DigitalValue value, bool force = false);
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

		// pin: 2-13
		// value: ARD_LOW or ARD_HIGH
		// the pins mode has to be set to ARD_OUTPUT or ARD_INPUT (in the latter mode pull-up resistors are enabled/disabled)
		// Note: pin 16-21 can also be used if analog inputs 0-5 are used as digital pins

		void sendPwm(int pin, int value, bool force = false);

		/// \brief Returns the last set PWM value (0-255) for the given pin
		///
		/// The pins mode has to be ARD_PWM
		///
		/// On the Arduino Uno the following pins are supported: 3, 5, 6, 9, 10 and 11
		/// \note Pin 16-21 can also be used if analog inputs 0-5 are used as digital pins
		int getPwm(int pin) const;

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

		void sendSysEx(MessageType command, vector <unsigned char> data);
		void sendSysEx(MessageType command);

		/// \brief Sends the `FIRMATA_START_SYSEX` command
		void sendSysExBegin();

		/// \brief Sends the `FIRMATA_END_SYSEX` command
		void sendSysExEnd();

		/// \brief Send a string to the Arduino
		/// \note Firmata can not handle strings longer than 12 characters.
		void sendString(string str);

		/// \brief This will cause your Arduino to reset and boot into the program again.
		void sendReset();

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

		/// \brief Useful for parsing SysEx messages
		int getValueFromTwo7bitBytes(unsigned char lsb, unsigned char msb);

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

		void setDigitalHistoryLength(int length);
		/// \brief Returns a pointer to the digital data history list for the
		/// given pin
		/// \note Pin 16-21 can also be used if analog inputs 0-5 are used as
		/// digital pins
		/// \param pin The pin number (2-13)
		list <int> * getDigitalHistory(int pin);

		void setAnalogHistoryLength(int length);
		/// \brief Returns a pointer to the analog data history list for the given pin.
		/// \param pin The Arduino Uno pin: 0-5
		list <int> * getAnalogHistory(int pin);

		void setStringHistoryLength(int length);
		/// \returns a pointer to the string history.
		list <string> * getStringHistory();

		void setSysExHistoryLength(int nSysEx);

		/// \returns a pointer to the SysEx history.
		list <vector <unsigned char> > * getSysExHistory();

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

		int getPinNum()
		{
			return _pin_capabilites.size();
		}

		using PinResolution = int;
		using PinCapability = map<PinMode, PinResolution>;
		PinCapability getPinCapibility(int pin)
		{
			if (pin >= _pin_capabilites.size())
				return map<PinMode, PinResolution>();
			else
			{
				return _pin_capabilites[pin];
			}
		}

		int get_analog_pin_from_digital(int digital_pin)
		{
			for (int i = 0; i < _analog_pins.size(); i++)
			{
				if (_analog_pins[i]->pinNum == digital_pin)
					return i;
			}
			return -1;
		}

	protected:
		mutable bool _initialized = false; ///\< \brief Indicate that pins are initialized.

		void sendProtocolVersionRequest();

		void sendFirmwareVersionRequest();
		// sets pin reporting to ARD_ON or ARD_OFF
		// enables / disables reporting for the pins port

		void sendDigitalPortReporting(int port, bool mode);
		// sets port reporting to ARD_ON or ARD_OFF
		// enables / disables reporting for ports 0-2
		// port 0: pins 2-7  (0,1 are serial RX/TX)
		// port 1: pins 8-13 (14,15 are disabled for the crystal)
		// port 2: pins 16-21 analog pins used as digital, all analog reporting will be turned off if this is set to ARD_ON

		void _processSerialData(unsigned char inputData);
		void _updateDigitalPort(int port, unsigned char value);
		void _updateAnalogPin(int pinNum, int value);
		virtual void _processSysExData(vector <unsigned char> data);

		// the last set servo values
		void sendByte(MessageType msg);
		void sendCapabilityQuery();
		void sendAnalogMappingQuery();
		void sendExtendedAnalog(int pin, int value);
		void sendPinStateQuery(int pin);

		void tryInit();
		void _printInfo();
		ofSerial _port;

		// --- history variables
		int _analogHistoryLength = 2;
		int _digitalHistoryLength = 2;
		int _stringHistoryLength = 1;
		int _sysExHistoryLength = 1;

		// --- data holders
		vector <unsigned char> _storedSerialData;
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

		vector<PinCapability> _pin_capabilites;

		struct DigitalPin
		{
			int pinNum = -1;
			PinMode mode = PinMode::_NULL;
			int value = -1;
			ReportStrategy reportStrategy = ReportStrategy::None;
			list <int> history;
		};

		struct DigitalPort
		{
			int value = 0;
			bool reporting = false;
		};

		vector<DigitalPin*> _analog_pins;
		vector<DigitalPin> _digital_pins;
		vector<DigitalPort> _digital_ports;

		bool _bUseDelay = true;
		mutable bool _connected = false; ///< \brief This yields true if a serial connection to Arduino exists.
		float _connectTime = 0.0f; ///< \brief This represents the (running) time of establishing a serial connection.
};

//typedef ofArduino ofStandardFirmata;

