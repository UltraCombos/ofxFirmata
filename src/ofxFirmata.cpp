/*
 * updated 2017 by chwan1@ultracombos.com
 *
 * 1/9/13:
 *   - Fixed issue where digitalPinchange
 *
 * 9/28/11:
 *   - updated to be Firmata 2.3/Arduino 1.0 compatible
 *   - fixed ability to use analog pins as digital inputs
 *
 * 3/5/11:
 *   - added servo support for firmata 2.2 and greater (should be
 *     backwards compatible with Erik Sjodin's older firmata servo
 *     implementation)
 *
 *
 * Copyright 2007-2008 (c) Erik Sjodin, eriksjodin.net
 *
 * Devloped at: The Interactive Institutet / Art and Technology,
 * OF Lab / Ars Electronica
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
 * included in all copies or substantial _portions of the Software.
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

#include "ofxFirmata.h"
#include "ofUtils.h"

 // TODO thread it?
 // TODO throw event or exception if the serial port goes down...
 //---------------------------------------------------------------------------
ofxFirmata::ofxFirmata() {
	_executeMultiByteCommand = 0x00; // 0x00 a pin mode (input), not a command in Firmata -> fail hard
	for (unsigned char & e : _storedInputData) {
		e = UCHAR_MAX;
	}
}

ofxFirmata::~ofxFirmata() {
	_port.close();
}

bool ofxFirmata::connect(const std::string & device, int baud) {
	_connectTime = ofGetElapsedTimef();
	_initialized = false;
	_port.listDevices();
	_connected = _port.setup(device.c_str(), baud);
	return _connected;
}

// this method is not recommended
// the preferred method is to listen for the EInitialized event in your application
bool ofxFirmata::isArduinoReady() const {
	if (_bUseDelay) {
		if (_initialized || (ofGetElapsedTimef() - _connectTime) > OF_ARDUINO_DELAY_LENGTH) {
			_connected = true;
		}
	}
	return _connected;
}

void ofxFirmata::setUseDelay(bool bDelay) {
	_bUseDelay = bDelay;
}

void ofxFirmata::setDigitalHistoryLength(int length) {
	if (length >= 2) {
		_digitalHistoryLength = length;
	}
}

void ofxFirmata::setAnalogHistoryLength(int length) {
	if (length >= 2) {
		_analogHistoryLength = length;
	}
}

void ofxFirmata::setSysExHistoryLength(int length) {
	if (length >= 1) {
		_sysExHistoryLength = length;
	}
}

void ofxFirmata::setStringHistoryLength(int length) {
	if (length >= 1) {
		_stringHistoryLength = length;
	}
}

void ofxFirmata::disconnect() {
	_port.close();
}

void ofxFirmata::update() {
	static float last_init_time = ofGetElapsedTimef();
	if (!_initialized && ofGetElapsedTimef() - last_init_time > 0.2)
	{
		last_init_time = ofGetElapsedTimef();
		tryInit();
	}
	vector <unsigned char> bytesToProcess;
	int bytesToRead = _port.available();
	if (bytesToRead > 0) {
		bytesToProcess.resize(bytesToRead);
		_port.readBytes(&bytesToProcess[0], bytesToRead);
		for (int i = 0; i < bytesToRead; i++) {
			processData((char)(bytesToProcess[i]));
		}
	}
}

int ofxFirmata::getAnalog(int pin) const {
	if (_analog_pins[pin].history.size() > 0) {
		return _analog_pins[pin].history.front();
	}
	else {
		return -1;
	}
}

int ofxFirmata::getDigital(int pinNum) const {
	if (pinNum >= _digital_pins.size())
		return -1;
	DigitalPin& pin = _digital_pins[pinNum];

	if (pin.mode == PinMode::DIGITAL_INPUT && pin.history.size() > 0) {
		return pin.history.front();
	}
	else if (pin.mode == PinMode::DIGITAL_OUTPUT) {
		return pin.value;
	}
	else {
		return -1;
	}
}

int ofxFirmata::getPwm(int pinNum) const {
	if (pinNum >= _digital_pins.size())
		return -1;

	if (_digital_pins[pinNum].mode == PinMode::PWM) {
		return _digital_pins[pinNum].value;
	}
	else {
		return -1;
	}
}

vector <unsigned char> ofxFirmata::getSysEx() const {
	return _sysExHistory.front();
}

string ofxFirmata::getString() const {
	return _stringHistory.front();
}

PinMode ofxFirmata::getDigitalPinMode(int pin) const {
	if (pin >= _digital_pins.size())
		return PinMode::DIGITAL_OUTPUT;

	return _digital_pins[pin].mode;
}

void ofxFirmata::sendDigital(int pinNum, DigitalValue value, bool force /*= false*/) {
	if (pinNum >= _digital_pins.size())
		return;

	DigitalPin& pin = _digital_pins[pinNum];

	if (pin.mode != PinMode::DIGITAL_INPUT && pin.mode != PinMode::DIGITAL_OUTPUT)
		return;

	if (pin.value == (int)value && !force)
		return;

	pin.value = (int)value;

	int port = pinNum / 8;
	int bit = pinNum - port * 8;

	// set the bit
	if (value == DigitalValue::HIGH) {
		_digital_ports[port].value |= (1 << bit);
	}
	else {
		_digital_ports[port].value &= ~(1 << bit);
	}

#if 0
	sendByte((int)MessageType::DIGITAL_IO_MESSAGE + port);
	sendValueAsTwo7bitBytes(_digitalPortValue[port]);
#else
	sendByte(MessageType::SET_DIGITAL_PIN_VALUE);
	sendByte(pinNum);
	sendByte((int)value);
#endif
}

void ofxFirmata::sendPwm(int pinNum, int value, bool force) {
	if (pinNum >= _digital_pins.size())
		return;

	DigitalPin& pin = _digital_pins[pinNum];

	if (pin.mode != PinMode::PWM)
		return;

	if (pin.value == value && !force)
		return;

	pin.value = value;

	sendExtendedAnalog(pinNum, value);
}

void ofxFirmata::sendSysEx(MessageType command, vector <unsigned char> data) {
	sendSysExBegin();
	sendByte(command);
	vector <unsigned char>::iterator it = data.begin();
	while (it != data.end()) {
		sendValueAsTwo7bitBytes(*it);
		it++;
	}
	sendSysExEnd();
}


void ofxFirmata::sendSysEx(MessageType command)
{
	sendByte(MessageType::START_SYSEX);
	sendByte(command);
	sendByte(MessageType::END_SYSEX);
}

void ofxFirmata::sendSysExBegin() {
	sendByte(MessageType::START_SYSEX);
}

void ofxFirmata::sendSysExEnd() {
	sendByte(MessageType::END_SYSEX);
}

void ofxFirmata::sendString(string str) {
	sendByte(MessageType::START_SYSEX);
	sendByte(MessageType::STRING_DATA);
	string::iterator it = str.begin();
	while (it != str.end()) {
		sendValueAsTwo7bitBytes(*it);
		it++;
	}
	sendByte(MessageType::END_SYSEX);
}

void ofxFirmata::sendProtocolVersionRequest() {
	sendByte(MessageType::PROTOCOL_VERSION);
}

void ofxFirmata::sendFirmwareVersionRequest() {
	sendSysEx(MessageType::REPORT_FIRMWARE);
}

void ofxFirmata::sendReset() {
	sendByte(MessageType::SYSTEM_RESET);
}

void ofxFirmata::sendAnalogPinReporting(int pinNum, bool reporting) {
	if (pinNum >= _analog_pins.size())
		return;

	int d_pin = _analog_pins[pinNum].digitalPinNum;
	DigitalPin& pin = _digital_pins[d_pin];

	// if this analog pin is set as a digital input, disable digital pin reporting
	if (pin.reporting == true) {
		sendDigitalPinReporting(d_pin, false);
	}

	pin.mode = PinMode::ANALOG_INPUT;

	sendByte((int)MessageType::REPORT_ANALOG_PIN + pinNum);
	sendByte(reporting);

	_analog_pins[pinNum].reporting = reporting;
}

void ofxFirmata::sendDigitalPinMode(int pin, PinMode mode) {
	sendByte(MessageType::SET_PIN_MODE);
	sendByte(pin);
	sendByte((int)mode);
	_digital_pins[pin].mode = mode;

	// turn on or off reporting on the port
	if (mode == PinMode::DIGITAL_INPUT) {
		sendDigitalPinReporting(pin, true);
	}
	else {
		sendDigitalPinReporting(pin, false);
	}
}

bool ofxFirmata::getAnalogPinReporting(int pin) const {
	if (pin <= _analog_pins.size())
	{
		ofLogWarning("");
		return false;
	}
	return _analog_pins[pin].reporting;
}

list <int> * ofxFirmata::getAnalogHistory(int pin) {
	return &_analog_pins[pin].history;
}

list <int> * ofxFirmata::getDigitalHistory(int pin) {
	return &_digital_pins[pin].history;
}

list <vector <unsigned char> > * ofxFirmata::getSysExHistory() {
	return &_sysExHistory;
}

list <string> * ofxFirmata::getStringHistory() {
	return &_stringHistory;
}

int ofxFirmata::getMajorProtocolVersion() const {
	return _majorFirmwareVersion;
}

int ofxFirmata::getMinorProtocolVersion() const {
	return _minorFirmwareVersion;
}

int ofxFirmata::getMajorFirmwareVersion() const {
	return _majorFirmwareVersion;
}

int ofxFirmata::getMinorFirmwareVersion() const {
	return _minorFirmwareVersion;
}

string ofxFirmata::getFirmwareName() const {
	return _firmwareName;
}

bool ofxFirmata::isInitialized() const {
	return _initialized;
}

// ------------------------------ private functions

void ofxFirmata::processData(unsigned char inputData) {

	char msg[100];
	sprintf(msg, "Received Byte: %i", inputData);
	//Logger::get("Application").information(msg);

	// we have command data
	if (_waitForData > 0 && inputData < 128) {
		_waitForData--;

		// collect the data
		_storedInputData[_waitForData] = inputData;

		// we have all data executeMultiByteCommand
		if (_waitForData == 0) {
			switch (_executeMultiByteCommand) {
			case MessageType::DIGITAL_IO_MESSAGE:
				processDigitalPort(_multiByteChannel, (_storedInputData[0] << 7) | _storedInputData[1]);
				break;

			case MessageType::PROTOCOL_VERSION:    // report version
				_majorProtocolVersion = _storedInputData[1];
				_minorProtocolVersion = _storedInputData[0];
				ofNotifyEvent(EProtocolVersionReceived, _majorProtocolVersion, this);
				break;

			case MessageType::ANALOG_IO_MESSAGE:
				if (_multiByteChannel >= _analog_pins.size())
					break;
				list <int>& history = _analog_pins[_multiByteChannel].history;
				if (history.size() > 0) {
					int previous = history.front();

					history.push_front((_storedInputData[0] << 7) | _storedInputData[1]);
					if ((int)history.size() > _analogHistoryLength) {
						history.pop_back();
					}

					// trigger an event if the pin has changed value
					if (history.front() != previous) {
						ofNotifyEvent(EAnalogPinChanged, _multiByteChannel, this);
					}
				}
				else {
					history.push_front((_storedInputData[0] << 7) | _storedInputData[1]);
					if ((int)history.size() > _analogHistoryLength) {
						history.pop_back();
					}
				}
				break;
			}
		}
	}
	// we have SysEx command data
	else if (_waitForData < 0) {

		// we have all sysex data
		if (inputData == (int)MessageType::END_SYSEX) {
			_waitForData = 0;
			processSysExData(_sysExData);
			_sysExData.clear();
		}
		// still have data, collect it
		else {
			_sysExData.push_back((unsigned char)inputData);
		}
	}
	// we have a command
	else {
		int command;

		// extract the command and channel info from a byte if it is less than 0xF0
		if (inputData < 0xF0) {
			command = inputData & 0xF0;
			_multiByteChannel = inputData & 0x0F;
		}
		else {
			// commands in the 0xF* range don't use channel data
			command = inputData;
		}

		switch (command) {
		case MessageType::PROTOCOL_VERSION:
		case MessageType::DIGITAL_IO_MESSAGE:
		case MessageType::ANALOG_IO_MESSAGE:
			_waitForData = 2;     // 2 bytes needed
			_executeMultiByteCommand = command;
			break;

		case MessageType::START_SYSEX:
			_sysExData.clear();
			_waitForData = -1;     // n bytes needed, -1 is used to indicate sysex message
			_executeMultiByteCommand = command;
			break;
		}
	}
}

// sysex data is assumed to be 8-bit bytes split into two 7-bit bytes.
void ofxFirmata::processSysExData(vector <unsigned char> data) {
	vector <unsigned char>::iterator it = data.begin();

	auto next_char = [&]()
	{
		if (it == data.end())
			return (unsigned char)127;

		unsigned char& c = *it;
		it++;
		return c;
	};

	// act on reserved sysEx messages (extended commands) or trigger SysEx event...
	switch (next_char()) {  //first byte in buffer is command
	case MessageType::REPORT_FIRMWARE:
	{
		_majorFirmwareVersion = next_char();
		_minorFirmwareVersion = next_char();
		string str;
		unsigned char buffer;

		while (it != data.end()) {
			buffer = next_char();
			buffer += next_char() << 7;
			str += buffer;
		}

		_firmwareName = str;

		_firmwareVersionSum = _majorFirmwareVersion * 10 + _minorFirmwareVersion;
		ofNotifyEvent(EFirmwareVersionReceived, _majorFirmwareVersion, this);
	}
	break;

	case MessageType::STRING_DATA:
	{
		string str;
		unsigned char buffer;

		while (it != data.end()) {
			buffer = next_char();
			buffer += next_char() << 7;
			str += buffer;
		}

		_stringHistory.push_front(str);
		if ((int)_stringHistory.size() > _stringHistoryLength) {
			_stringHistory.pop_back();
		}

		ofNotifyEvent(EStringReceived, str, this);
	}
	break;
	case MessageType::CAPABILITY_RESPONSE:
	{
		_pin_capabilites.clear();
		_pin_capabilites.emplace_back(map<PinMode, int>());
		while (it != data.end())
		{
			auto c = next_char();
			if (c == 0x7f)
			{
				_pin_capabilites.emplace_back(map<PinMode, int>());
			}
			else
			{
				PinMode pinmode = (PinMode)c;
				int resolution = 1 << (next_char() - 1);
				_pin_capabilites.back()[pinmode] = resolution;
			}
		}
		_pin_capabilites.pop_back();
	}
	break;
	case MessageType::ANALOG_MAPPING_RESPONSE:
	{
		_analog_pins.clear();
		int idx = 0;
		while (it != data.end())
		{
			auto c = next_char();
			if (c == 127)
			{
			}
			else
			{
				_analog_pins.resize(c + 1);
				_analog_pins[c].digitalPinNum = idx;
			}
			idx++;
		}
	}
	break;
	case MessageType::PIN_STATE_RESPONSE:
	{
		int pin = next_char();
		PinMode mode = (PinMode)next_char();
		if (pin <= _digital_pins.size())
		{
			_digital_pins[pin].mode = mode;
		}
		while (it != data.end())
		{
			next_char();
		}

	}
	break;
	default:    // the message isn't in Firmatas extended command set
		_sysExHistory.push_front(data);
		if ((int)_sysExHistory.size() > _sysExHistoryLength) {
			_sysExHistory.pop_back();
		}
		ofNotifyEvent(ESysExReceived, data, this);
		break;
	}
}

void ofxFirmata::processDigitalPort(int port, unsigned char value) {
	unsigned char mask;
	int previous;

	for (int i = port * 8; i < port * 8 + 8; ++i) {
		previous = -1;

		if (i >= _digital_pins.size())
			continue;

		DigitalPin& pin = _digital_pins[i];

		if (pin.mode == PinMode::DIGITAL_INPUT) {
			if (pin.history.size() > 0) {
				previous = pin.history.front();
			}

			mask = 1 << i;
			pin.history.push_front((value & mask) >> i);

			if ((int)pin.history.size() > _digitalHistoryLength) {
				pin.history.pop_back();
			}

			// trigger an event if the pin has changed value
			if (pin.history.front() != previous) {
				ofNotifyEvent(EDigitalPinChanged, i, this);
			}
		}
	}
}

void ofxFirmata::sendDigitalPortReporting(int port, bool reporting) {
	sendByte((int)MessageType::REPORT_DIGITAL_PORT + port);
	sendByte(reporting);
	_digital_ports[port].reporting = reporting;

	//TODO check necessity of these code
#if 0
	int offset;

	if (_firmwareVersionSum >= FIRMWARE2_3) {
		offset = 2;
	}
	else {
		offset = 0;
	}

	// for Firmata 2.3 and higher:
	if (port == 1 && mode == ARD_ON) {
		for (int i = 0; i < 2; i++) {
			_analogPinReporting[i] = ARD_OFF;
		}
	}

	// for Firmata 2.3 and all prior Firmata protocol versions:
	if (port == 2 && mode == ARD_ON) { // if reporting is turned on on port 2 then ofArduino on the Arduino disables all analog reporting
		for (int i = offset; i < ARD_TOTAL_ANALOG_PINS; i++) {
			_analogPinReporting[i] = ARD_OFF;
		}
	}
#endif
}

void ofxFirmata::sendDigitalPinReporting(int pin, bool reporting) {
	_digital_pins[pin].reporting = reporting;

	int port = pin / 8;

	if (reporting == true) {
		sendDigitalPortReporting(port, true);
	}
	else {
		bool send = true;
		for (int i = port * 8; i < port * 8 + 8; ++i) {
			if (_digital_pins[i].reporting) {
				send = false;
			}
		}
		if (send) {
			sendDigitalPortReporting(port, false);
		}
	}
}

void ofxFirmata::sendByte(unsigned char byte) {
	//char msg[100];
	//sprintf(msg, "Sending Byte: %i", byte);
	//Logger::get("Application").information(msg);
	_port.writeByte(byte);
}


void ofxFirmata::sendByte(MessageType msg)
{
	sendByte((int)msg);
}

void ofxFirmata::sendCapabilityQuery()
{
	sendSysEx(MessageType::CAPABILITY_QUERY);
}

void ofxFirmata::sendAnalogMappingQuery()
{
	sendSysEx(MessageType::ANALOG_MAPPING_QUERY);
}

void ofxFirmata::sendExtendedAnalog(int pin, int value)
{
	sendSysExBegin();
	sendByte(MessageType::EXTENDED_ANALOG);
	sendByte(pin);
	sendValueAsTwo7bitBytes(value);
	sendSysExEnd();
}

void ofxFirmata::sendPinStateQuery(int pin)
{
	sendSysExBegin();
	sendByte(MessageType::PIN_STATE_QUERY);
	sendByte(pin);
	sendSysExEnd();
}

void ofxFirmata::tryInit()
{
	if (_minorFirmwareVersion == 0 && _majorFirmwareVersion == 0)
		sendFirmwareVersionRequest();

	if (_pin_capabilites.size() == 0)
		sendCapabilityQuery();

	if (_analog_pins.size() == 0)
		sendAnalogMappingQuery();

	if (_minorFirmwareVersion != 0 &&
		_majorFirmwareVersion != 0 &&
		_pin_capabilites.size() != 0 &&
		_analog_pins.size() != 0)
	{
		if (_digital_pins.size() != _pin_capabilites.size())
		{
			_digital_pins.resize(_pin_capabilites.size());
			// ports
			int port_count = _pin_capabilites.size() / 8 + 1;
			_digital_ports.resize(port_count);
		}
		bool init_finished = true;
		for (int i = 0; i < _pin_capabilites.size(); i++)
		{
			if (_digital_pins[i].mode == PinMode::_NULL)
			{
				init_finished = false;
				sendPinStateQuery(i);
			}
		}
		if (init_finished == true)
		{
			printf("===================== Firmata Initialized =====================\n");
			//report states
			{
				// print firmware name and version to the console
				cout << getFirmwareName() << endl;
				cout << "firmata v" << getMajorFirmwareVersion() << "." << getMinorFirmwareVersion() << endl;

				printf("Pin Capability:\n");
				for (int i = 0; i < _pin_capabilites.size(); i++)
				{
					printf("\tpin %2d: ", i);
					for each (auto& pair in _pin_capabilites[i])
					{
						cout << pinModeToString(pair.first) << "[" << pair.second << "] ";
					}
					cout << endl;
				}
				printf("\nAnalog Mapping:\n");
				for (int i = 0; i < _analog_pins.size(); i++)
				{
					printf("\ta[%2d] = d[%2d]\n", i, _analog_pins[i].digitalPinNum);
				}
				printf("\nPin State:\n");
				for (size_t i = 0; i < _digital_pins.size(); i++)
				{
					PinMode mode = _digital_pins[i].mode;
					printf("\tpin %2d: %s\n", i, pinModeToString(mode).c_str());
				}
				printf("===============================================================\n\n\n");
			}

			_initialized = true;
			ofNotifyEvent(EInitialized, _majorFirmwareVersion, this);
		}
	}
}

std::string ofxFirmata::pinModeToString(PinMode mode)
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

// in Firmata (and MIDI) data bytes are 7-bits. The 8th bit serves as a flag to mark a byte as either command or data.
// therefore you need two data bytes to send 8-bits (a char).
void ofxFirmata::sendValueAsTwo7bitBytes(int value) {
	sendByte(value & 127); // LSB
	sendByte(value >> 7 & 127); // MSB
}

// SysEx data is sent as 8-bit bytes split into two 7-bit bytes, this function merges two 7-bit bytes back into one 8-bit byte.
int ofxFirmata::getValueFromTwo7bitBytes(unsigned char lsb, unsigned char msb) {
	return (msb << 7) | lsb;
}

void ofxFirmata::sendServo(int pin, int value, bool force) {
	// for firmata v2.2 and greater
	if (_firmwareVersionSum >= FIRMWARE2_2) {
		if (_digital_pins[pin].mode == PinMode::SERVO && (_digital_pins[pin].value != value || force)) {
			sendExtendedAnalog(pin, value);
			_digital_pins[pin].value = value;
		}
	}
}

// angle parameter is no longer supported. keeping for backwards compatibility
void ofxFirmata::sendServoConfig(int pin, int minPulse /*= 544*/, int maxPulse /*= 2400*/) {
	sendByte(MessageType::START_SYSEX);
	sendByte(MessageType::SERVO_CONFIG);
	sendByte(pin);
	sendValueAsTwo7bitBytes(minPulse);
	sendValueAsTwo7bitBytes(maxPulse);
	sendByte(MessageType::END_SYSEX);
	_digital_pins[pin].mode = PinMode::SERVO;
}

int ofxFirmata::getServo(int pin) const {
	if (_digital_pins[pin].mode != PinMode::SERVO)
		return -1;

	// for firmata v2.2 and greater
	if (_firmwareVersionSum >= FIRMWARE2_2) {
		return _digital_pins[pin].value;
	}
	// for versions prior to 2.2
	else {
		return -1;
	}
}
