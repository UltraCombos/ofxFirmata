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

const string ToString(PinMode value)
{
	static std::map<PinMode, string> table;
	static bool isInit = false;
	if (isInit)
		return table[value];

#define X(a, b, c) table[a] = c;
	VALUE_TABLE
#undef X

		isInit = true;
	return table[value];
}

const PinMode FromString(string value)
{
	static std::map<string, PinMode> table;
	static bool isInit = false;
	if (isInit)
		return table[value];

#define X(a, b, c) table[c] = a;
	VALUE_TABLE
#undef X

		isInit = true;
	return table[value];
}

 // TODO thread it?
 // TODO throw event or exception if the serial port goes down...
 //---------------------------------------------------------------------------
ofxFirmata::ofxFirmata() {}

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
			_processSerialData((char)(bytesToProcess[i]));
		}
	}
	for (int i = 0; i < _digital_ports.size(); i++)
	{
		bool reporting = _digital_ports[i].reporting;
		if (reporting)
		{
			sendDigitalPortReporting(i, reporting);
			sendDigitalPortReporting(i, reporting);//send twice to for update in the same frame
		}
	}
	for (int i = 0; i < _analog_pins.size(); i++)
	{
		list <int>& history = _analog_pins[i]->history;
		if (history.size() == 0)
			continue;

		if (_analog_pins[i]->reportStrategy == ReportStrategy::Always && _analog_pins[i]->mode == PinMode::ANALOG_INPUT) {
			ofNotifyEvent(EAnalogPinChanged, i, this);
		}
	}
	for (int i = 0; i < _digital_pins.size(); i++)
	{
		list <int>& history = _digital_pins[i].history;
		if (history.size() == 0)
			continue;

		if (_digital_pins[i].reportStrategy == ReportStrategy::Always && _digital_pins[i].mode == PinMode::DIGITAL_INPUT) {
			ofNotifyEvent(EDigitalPinChanged, i, this);
		}
	}
}

int ofxFirmata::getAnalog(int pin) const {
	if (_analog_pins[pin]->history.size() > 0) {
		return _analog_pins[pin]->history.front();
	}
	else {
		return -1;
	}
}

int ofxFirmata::getDigital(int pinNum) const {
	if (pinNum >= _digital_pins.size())
		return -1;
	DigitalPin pin = _digital_pins[pinNum];

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

void ofxFirmata::sendAnalogPinReporting(int pinNum, ReportStrategy reportStrategy, bool force /*=false*/) {
	if (pinNum >= _analog_pins.size() || _initialized == false)
		return;

	if (_analog_pins[pinNum]->reportStrategy == reportStrategy && !force)
		return;

	DigitalPin& pin = *_analog_pins[pinNum];

	// if this analog pin is set as a digital input, disable digital pin reporting
	if (pin.reportStrategy != ReportStrategy::None) {
		sendDigitalPinReporting(pin.pinNum, ReportStrategy::None);
	}

	pin.mode = PinMode::ANALOG_INPUT;

	sendByte((int)MessageType::REPORT_ANALOG_PIN + pinNum);
	if(reportStrategy != ReportStrategy::None)
		sendByte(1);
	else
		sendByte(0);

	_analog_pins[pinNum]->reportStrategy = reportStrategy;
}

void ofxFirmata::sendDigitalPinReporting(int pin, ReportStrategy reportStrategy) {
	ofLogWarning("ofxFirmata") << "sendDigitalPinReporting() is not working correctly" << endl;
	_digital_pins[pin].reportStrategy = reportStrategy;

	sendDigitalPinMode(pin, PinMode::DIGITAL_INPUT);

	int port = pin / 8;

	//sendPinStateQuery(pin);

	if (reportStrategy != ReportStrategy::None) {
		sendDigitalPortReporting(port, true);
	}
	else {
		bool send = true;
		for (int i = port * 8; i < port * 8 + 8; ++i) {
			if (_digital_pins[i].reportStrategy != ReportStrategy::None) {
				send = false;
			}
		}
		if (send) {
			sendDigitalPortReporting(port, false);
		}
	}
}

void ofxFirmata::sendDigitalPinMode(int pin, PinMode mode, bool force /*=false*/) {
	if (pin >= _digital_pins.size())
		return;

	if (_digital_pins[pin].mode == mode && !force)
		return;

	sendByte(MessageType::SET_PIN_MODE);
	sendByte(pin);
	sendByte((int)mode);
	_digital_pins[pin].mode = mode;

// 	// turn on or off reporting on the port
// 	if (mode == PinMode::DIGITAL_INPUT) {
// 		sendDigitalPinReporting(pin, true);
// 	}
// 	else {
// 		sendDigitalPinReporting(pin,  ReportStrategy::None);
// 	}
}

ofxFirmata::ReportStrategy ofxFirmata::getAnalogPinReporting(int pin) const {
	if (pin <= _analog_pins.size())
	{
		ofLogWarning("");
		return ReportStrategy::None;
	}
	return _analog_pins[pin]->reportStrategy;
}

list <int> * ofxFirmata::getAnalogHistory(int pin) {
	return &_analog_pins[pin]->history;
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

void ofxFirmata::_processSerialData(unsigned char inputData) {
	//char msg[100];
	//sprintf(msg, "Received Byte: %i", inputData);
	//Logger::get("Application").information(msg);

	if (_storedSerialData.size() == 0)
	{
		int command;
		int channel_data;
		// extract the command and channel info from a byte if it is less than 0xF0
		if (inputData < (int)MessageType::START_SYSEX && inputData >= (int)MessageType::DIGITAL_IO_MESSAGE)
		{
			command = inputData & 0xF0;
			channel_data = inputData & 0x0F;
		}
		else
			command = inputData;

		switch (command)
		{
		case MessageType::DIGITAL_IO_MESSAGE:
		case MessageType::ANALOG_IO_MESSAGE:
			_storedSerialData.push_back(command);
			_storedSerialData.push_back(channel_data);
			break;
		case MessageType::START_SYSEX:
		case MessageType::PROTOCOL_VERSION:
			_storedSerialData.push_back(command);
			break;
		default:
			break;
		}
	}
	else
	{
		switch (_storedSerialData.front())
		{
		case MessageType::DIGITAL_IO_MESSAGE:
			if (_storedSerialData.size() < 4)
			{
				if (inputData < 128)
					_storedSerialData.push_back(inputData);
			}
			else
			{
				int pinNum = _storedSerialData[1];
				int value = getValueFromTwo7bitBytes(_storedSerialData[2], _storedSerialData[3]);
				_updateDigitalPort(pinNum, value);
				_storedSerialData.clear();
			}
			break;
		case MessageType::ANALOG_IO_MESSAGE:
			if (_storedSerialData.size() < 4)
			{
				if (inputData < 128)
					_storedSerialData.push_back(inputData);
			}
			else
			{
				int pinNum = _storedSerialData[1];
				int value = getValueFromTwo7bitBytes(_storedSerialData[2], _storedSerialData[3]);
				_updateAnalogPin(pinNum, value);
				_storedSerialData.clear();
			}
			break;
		case MessageType::START_SYSEX:
			if (inputData != (int)MessageType::END_SYSEX)
			{
				_storedSerialData.push_back(inputData);
			}
			else
			{
				_processSysExData(_storedSerialData);
				_storedSerialData.clear();
			}
			break;
		case MessageType::PROTOCOL_VERSION:
			if (_storedSerialData.size() < 3)
			{
				if (inputData < 128)
					_storedSerialData.push_back(inputData);
			}
			else
			{
				_majorProtocolVersion = _storedSerialData[1];
				_minorProtocolVersion = _storedSerialData[2];
				ofNotifyEvent(EProtocolVersionReceived, _majorProtocolVersion, this);
				_storedSerialData.clear();
			}
			break;
		default:
			break;
		}
	}
}

// sysex data is assumed to be 8-bit bytes split into two 7-bit bytes.
void ofxFirmata::_processSysExData(vector <unsigned char> data) {
	vector <unsigned char>::iterator it = data.begin();

	auto next_char = [&]()
	{
		if (it == data.end())
			return (unsigned char)127;

		unsigned char& c = *it;
		it++;
		return c;
	};

	if (next_char() != (int)MessageType::START_SYSEX)
		return;

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
				_analog_pins[c] = &_digital_pins[idx];
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
			_digital_pins[pin].pinNum = pin;
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

void ofxFirmata::_updateDigitalPort(int port, unsigned char value) {
	unsigned char mask;
	int previous;

	//printf("_updateDigitalPort %d %s\n", port, ofToBinary(value).c_str());

	for (int i = port * 8; i < port * 8 + 8; ++i) {
		previous = -1;

		if (i >= _digital_pins.size())
			continue;

		DigitalPin& pin = _digital_pins[i];

		if (pin.mode == PinMode::DIGITAL_INPUT) 
		{
			if (pin.history.size() > 0) {
				previous = pin.history.front();
			}
			int shift = (i % 8);
			mask = 1 << shift;
			int v = (value & mask) >> shift;
			pin.history.push_front(v);

			if ((int)pin.history.size() > _digitalHistoryLength) {
				pin.history.pop_back();
			}

			// trigger an event if the pin has changed value
			if (_digital_pins[i].reportStrategy == ReportStrategy::OnChange  && pin.history.front() != previous) {
				ofNotifyEvent(EDigitalPinChanged, i, this);
			}
		}
	}
}

void ofxFirmata::sendDigitalPortReporting(int port, bool reporting) {
	sendByte((int)MessageType::REPORT_DIGITAL_PORT + port);
	sendByte(reporting);
	_digital_ports[port].reporting = reporting;

	//printf("sendDigitalPortReporting port%d report%d\n", port, (int)reporting);
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
	{
		sendFirmwareVersionRequest();
		return;
	}

	if (_pin_capabilites.size() == 0)
	{
		sendCapabilityQuery();
		return;
	}

	if (_digital_pins.size() != _pin_capabilites.size())
	{
		_digital_pins.resize(_pin_capabilites.size());
		// ports
		int port_count = (_pin_capabilites.size() + 7) / 8;
		_digital_ports.resize(port_count);
		return;
	}

	if (_analog_pins.size() == 0)
	{
		sendAnalogMappingQuery();
		return;
	}

	{
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
			_printInfo();
			_initialized = true;
			ofNotifyEvent(EInitialized, _majorFirmwareVersion, this);
		}
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

void ofxFirmata::_updateAnalogPin(int pinNum, int value)
{
	if (pinNum >= _analog_pins.size() || _analog_pins[pinNum] == nullptr)
		return;

	list <int>& history = _analog_pins[pinNum]->history;
	if (history.size() > 0) {
		int previous = history.front();
		if (_analog_pins[pinNum]->reportStrategy == ReportStrategy::OnChange && value != previous) {
			ofNotifyEvent(EAnalogPinChanged, pinNum, this);
		}
	}
	history.push_front(value);

	if ((int)history.size() > _analogHistoryLength) {
		history.pop_back();
	}
}

void ofxFirmata::_printInfo()
{
	printf("===================== Firmata Initialized =====================\n");
	cout << getFirmwareName() << endl;
	cout << "firmata v" << getMajorFirmwareVersion() << "." << getMinorFirmwareVersion() << endl;

	printf("Pin Capability:\n");
	for (int i = 0; i < _pin_capabilites.size(); i++)
	{
		printf("\tpin %2d: ", i);
		for each (auto& pair in _pin_capabilites[i])
		{
			printf("%s[%d] ", ToString(pair.first).c_str(), pair.second);
		}
		cout << endl;
	}
	printf("\nAnalog Mapping:\n");
	for (int i = 0; i < _analog_pins.size(); i++)
	{
		printf("\ta[%2d] = d[%2d]\n", i, _analog_pins[i]->pinNum);
	}
	printf("\nPin State:\n");
	for (size_t i = 0; i < _digital_pins.size(); i++)
	{
		PinMode mode = _digital_pins[i].mode;
		printf("\tpin %2d: %s\n", i, ToString(mode).c_str());
	}
	printf("===============================================================\n\n\n");
}
