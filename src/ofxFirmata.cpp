/*
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
ofxFirmata::ofxFirmata(){
	_executeMultiByteCommand = 0x00; // 0x00 a pin mode (input), not a command in Firmata -> fail hard
	for(unsigned char & e : _storedInputData){
		e = UCHAR_MAX;
	}
// 	for(int & e : _digitalPinMode){
// 		e = INT_MAX;
// 	}
	for(int & e : _digitalPinValue){
		e = INT_MAX;
	}
	for(int & e : _digitalPortValue){
		e = INT_MAX;
	}
	for(int & e : _digitalPortReporting){
		e = INT_MAX;
	}
	for(int & e : _digitalPinReporting){
		e = INT_MAX;
	}
	for(int & e : _analogPinReporting){
		e = INT_MAX;
	}
	for(int & e : _servoValue){
		e = INT_MAX;
	}
}

ofxFirmata::~ofxFirmata(){
	_port.close();
}

// initialize pins once we get the Firmata version back from the Arduino board
// the version is sent automatically by the Arduino board on startup
void ofxFirmata::initPins() const {
	int firstAnalogPin;

	if(_initialized){
		return;                 // already initialized
	}

	// support Firmata 2.3/Arduino 1.0 with backwards compatibility
	// to previous protocol versions
	if(_firmwareVersionSum >= FIRMWARE2_3){
		_totalDigitalPins = 20;
		firstAnalogPin = 14;
	}else{
		_totalDigitalPins = ARD_TOTAL_DIGITAL_PINS;
		firstAnalogPin = 16;
	}

	// ports
	for(int i = 0; i < ARD_TOTAL_PORTS; ++i){
		_digitalPortValue[i] = 0;
		_digitalPortReporting[i] = ARD_OFF;
	}

	// digital pins
	for(int i = 0; i < firstAnalogPin; ++i){
		_digitalPinValue[i] = -1;
		_digitalPinMode[i] = PinMode::DIGITAL_OUTPUT;
		_digitalPinReporting[i] = ARD_OFF;
	}

	// analog in pins
	for(int i = firstAnalogPin; i < _totalDigitalPins; ++i){
		_analogPinReporting[i - firstAnalogPin] = ARD_OFF;
		// analog pins used as digital
		_digitalPinMode[i] = PinMode::ANALOG_INPUT;
		_digitalPinValue[i] = -1;
	}

	for(int i = 0; i < _totalDigitalPins; ++i){
		_servoValue[i] = -1;
	}

	_initialized = true;
}

bool ofxFirmata::connect(const std::string & device, int baud){
	connectTime = ofGetElapsedTimef();
	_initialized = false;
	_port.listDevices();
	connected = _port.setup(device.c_str(), baud);
	return connected;
}

// this method is not recommended
// the preferred method is to listen for the EInitialized event in your application
bool ofxFirmata::isArduinoReady() const {
	if(bUseDelay){
		if(_initialized || (ofGetElapsedTimef() - connectTime) > OF_ARDUINO_DELAY_LENGTH){
			initPins();
			connected = true;
		}
	}
	return connected;
}

void ofxFirmata::setUseDelay(bool bDelay){
	bUseDelay = bDelay;
}

void ofxFirmata::setDigitalHistoryLength(int length){
	if(length >= 2){
		_digitalHistoryLength = length;
	}
}

void ofxFirmata::setAnalogHistoryLength(int length){
	if(length >= 2){
		_analogHistoryLength = length;
	}
}

void ofxFirmata::setSysExHistoryLength(int length){
	if(length >= 1){
		_sysExHistoryLength = length;
	}
}

void ofxFirmata::setStringHistoryLength(int length){
	if(length >= 1){
		_stringHistoryLength = length;
	}
}

void ofxFirmata::disconnect(){
	_port.close();
}

void ofxFirmata::update(){
	vector <unsigned char> bytesToProcess;
	int bytesToRead = _port.available();
	if(bytesToRead > 0){
		bytesToProcess.resize(bytesToRead);
		_port.readBytes(&bytesToProcess[0], bytesToRead);
		for(int i = 0; i < bytesToRead; i++){
			processData((char)(bytesToProcess[i]));
		}
	}
}

int ofxFirmata::getAnalog(int pin) const {
	if(_analogHistory[pin].size() > 0){
		return _analogHistory[pin].front();
	}else{
		return -1;
	}
}

int ofxFirmata::getDigital(int pin) const {
	if(_digitalPinMode[pin] == PinMode::DIGITAL_INPUT && _digitalHistory[pin].size() > 0){
		return _digitalHistory[pin].front();
	}else if(_digitalPinMode[pin] == PinMode::DIGITAL_OUTPUT){
		return _digitalPinValue[pin];
	}else{
		return -1;
	}
}

int ofxFirmata::getPwm(int pin) const {
	if(_digitalPinMode[pin] == PinMode::PWM){
		return _digitalPinValue[pin];
	}else{
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
	return _digitalPinMode[pin];
}

void ofxFirmata::sendDigital(int pin, int value, bool force){
	if((_digitalPinMode[pin] == PinMode::DIGITAL_INPUT || _digitalPinMode[pin] == PinMode::DIGITAL_OUTPUT) && (_digitalPinValue[pin] != value || force)){

		_digitalPinValue[pin] = value;

		int port = 0;
		int bit = 0;
		int port1Offset;
		int port2Offset;

		// support Firmata 2.3/Arduino 1.0 with backwards compatibility
		// to previous protocol versions
		if(_firmwareVersionSum >= FIRMWARE2_3){
			port1Offset = 16;
			port2Offset = 20;
		}else{
			port1Offset = 14;
			port2Offset = 22;
		}

		if(pin < 8 && pin > 1){
			port = 0;
			bit = pin;
		}else if(pin > 7 && pin < port1Offset){
			port = 1;
			bit = pin - 8;
		}else if(pin > 15 && pin < port2Offset){
			port = 2;
			bit = pin - 16;
		}

		// set the bit
		if(value == 1){
			_digitalPortValue[port] |= (1 << bit);
		}

		// clear the bit
		if(value == 0){
			_digitalPortValue[port] &= ~(1 << bit);
		}

		sendByte((int)MessageType::DIGITAL_IO_MESSAGE + port);
		sendValueAsTwo7bitBytes(_digitalPortValue[port]);
	}
}

void ofxFirmata::sendPwm(int pin, int value, bool force){
	if(_digitalPinMode[pin] == PinMode::PWM && (_digitalPinValue[pin] != value || force)){
		sendByte((int)MessageType::ANALOG_IO_MESSAGE + pin);
		sendValueAsTwo7bitBytes(value);
		_digitalPinValue[pin] = value;
	}
}

void ofxFirmata::sendSysEx(MessageType command, vector <unsigned char> data){
	sendSysExBegin();
	sendByte(command);
	vector <unsigned char>::iterator it = data.begin();
	while(it != data.end()){
		sendValueAsTwo7bitBytes(*it);
		it++;
	}
	sendSysExEnd();
}

void ofxFirmata::sendSysExBegin(){
	sendByte(MessageType::START_SYSEX);
}

void ofxFirmata::sendSysExEnd(){
	sendByte(MessageType::END_SYSEX);
}

void ofxFirmata::sendString(string str){
	sendByte(MessageType::START_SYSEX);
	sendByte(MessageType::STRING_DATA);
	string::iterator it = str.begin();
	while(it != str.end()){
		sendValueAsTwo7bitBytes(*it);
		it++;
	}
	sendByte(MessageType::END_SYSEX);
}

void ofxFirmata::sendProtocolVersionRequest(){
	sendByte(MessageType::PROTOCOL_VERSION);
}

void ofxFirmata::sendFirmwareVersionRequest(){
	sendSysEx(MessageType::REPORT_FIRMWARE);
}

void ofxFirmata::sendReset(){
	sendByte(MessageType::SYSTEM_RESET);
}

void ofxFirmata::sendAnalogPinReporting(int pin, int mode){

	int firstAnalogPin;
	// support Firmata 2.3/Arduino 1.0 with backwards compatibility
	// to previous protocol versions
	if(_firmwareVersionSum >= FIRMWARE2_3){
		firstAnalogPin = 14;
	}else{
		firstAnalogPin = 16;
	}

	// if this analog pin is set as a digital input, disable digital pin reporting
	if(_digitalPinReporting[pin + firstAnalogPin] == ARD_ON){
		sendDigitalPinReporting(pin + firstAnalogPin, ARD_OFF);
	}

	_digitalPinMode[firstAnalogPin + pin] = PinMode::ANALOG_INPUT;

	sendByte((int)MessageType::REPORT_ANALOG_PIN + pin);
	sendByte(mode);
	_analogPinReporting[pin] = mode;
}

void ofxFirmata::sendDigitalPinMode(int pin, PinMode mode){
	sendByte(MessageType::SET_PIN_MODE);
	sendByte(pin);
	sendByte((int)mode);
	_digitalPinMode[pin] = mode;

	// turn on or off reporting on the port
	if(mode == PinMode::DIGITAL_INPUT){
		//sendDigitalPinReporting(pin, ARD_ON);
	}else{
		//sendDigitalPinReporting(pin, ARD_OFF);
	}
}

int ofxFirmata::getAnalogPinReporting(int pin) const {
	return _analogPinReporting[pin];
}

list <int> * ofxFirmata::getAnalogHistory(int pin){
	return &_analogHistory[pin];
}

list <int> * ofxFirmata::getDigitalHistory(int pin){
	return &_digitalHistory[pin];
}

list <vector <unsigned char> > * ofxFirmata::getSysExHistory(){
	return &_sysExHistory;
}

list <string> * ofxFirmata::getStringHistory(){
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

void ofxFirmata::processData(unsigned char inputData){

	char msg[100];
	sprintf(msg, "Received Byte: %i", inputData);
	//Logger::get("Application").information(msg);

	// we have command data
	if(_waitForData > 0 && inputData < 128){
		_waitForData--;

		// collect the data
		_storedInputData[_waitForData] = inputData;

		// we have all data executeMultiByteCommand
		if(_waitForData == 0){
			switch(_executeMultiByteCommand){
			 case MessageType::DIGITAL_IO_MESSAGE:
				 processDigitalPort(_multiByteChannel, (_storedInputData[0] << 7) | _storedInputData[1]);
				 break;

			 case MessageType::PROTOCOL_VERSION:    // report version
				 _majorProtocolVersion = _storedInputData[1];
				 _minorProtocolVersion = _storedInputData[0];
				 ofNotifyEvent(EProtocolVersionReceived, _majorProtocolVersion, this);
				 break;

			 case MessageType::ANALOG_IO_MESSAGE:
				 if(_analogHistory[_multiByteChannel].size() > 0){
					 int previous = _analogHistory[_multiByteChannel].front();

					 _analogHistory[_multiByteChannel].push_front((_storedInputData[0] << 7) | _storedInputData[1]);
					 if((int)_analogHistory[_multiByteChannel].size() > _analogHistoryLength){
						 _analogHistory[_multiByteChannel].pop_back();
					 }

					 // trigger an event if the pin has changed value
					 if(_analogHistory[_multiByteChannel].front() != previous){
						 ofNotifyEvent(EAnalogPinChanged, _multiByteChannel, this);
					 }
				 }else{
					 _analogHistory[_multiByteChannel].push_front((_storedInputData[0] << 7) | _storedInputData[1]);
					 if((int)_analogHistory[_multiByteChannel].size() > _analogHistoryLength){
						 _analogHistory[_multiByteChannel].pop_back();
					 }
				 }
				 break;
			}

		}
	}
	// we have SysEx command data
	else if(_waitForData < 0){

		// we have all sysex data
		if(inputData == (int)MessageType::END_SYSEX){
			_waitForData = 0;
			processSysExData(_sysExData);
			_sysExData.clear();
		}
		// still have data, collect it
		else{
			_sysExData.push_back((unsigned char)inputData);
		}
	}
	// we have a command
	else{

		int command;

		// extract the command and channel info from a byte if it is less than 0xF0
		if(inputData < 0xF0){
			command = inputData & 0xF0;
			_multiByteChannel = inputData & 0x0F;
		}else{
			// commands in the 0xF* range don't use channel data
			command = inputData;
		}

		switch(command){
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
void ofxFirmata::processSysExData(vector <unsigned char> data){

	string str;

	vector <unsigned char>::iterator it = data.begin();

	auto next_char = [&]()
	{
		if (it == data.end())
			return (unsigned char)127;

		unsigned char& c = *it;
		it++;
		return c;
	};

	unsigned char buffer;

	// act on reserved sysEx messages (extended commands) or trigger SysEx event...
	switch (next_char()) {  //first byte in buffer is command
	case MessageType::REPORT_FIRMWARE:
		_majorFirmwareVersion = next_char();
		_minorFirmwareVersion = next_char();

		while (it != data.end()) {
			buffer = next_char();
			buffer += next_char() << 7;
			str += buffer;
		}

		_firmwareName = str;

		_firmwareVersionSum = _majorFirmwareVersion * 10 + _minorFirmwareVersion;
		ofNotifyEvent(EFirmwareVersionReceived, _majorFirmwareVersion, this);

		// trigger the initialization event
		if (!_initialized) {
			initPins();
			ofNotifyEvent(EInitialized, _majorFirmwareVersion, this);
		}
		sendCapabilityQuery();
		break;

	case MessageType::STRING_DATA:
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
				printf("pin %d: mode[%d] resolution[%d]\n", _pin_capabilites.size() - 1, (int)pinmode, resolution);
				_pin_capabilites.back()[pinmode] = resolution;
			}
		}
		_pin_capabilites.pop_back();
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

void ofxFirmata::processDigitalPort(int port, unsigned char value){

	unsigned char mask;
	int previous;
	int i;
	int pin;
	int port1Pins;
	int port2Pins;

	// support Firmata 2.3/Arduino 1.0 with backwards compatibility to previous protocol versions
	if(_firmwareVersionSum >= FIRMWARE2_3){
		port1Pins = 8;
		port2Pins = 4;
	}else{
		port1Pins = 6;
		port2Pins = 6;
	}

	switch(port){
	 case 0: // pins 2-7  (0,1 are ignored as serial RX/TX)
		 for(i = 2; i < 8; ++i){
			 pin = i;
			 previous = -1;
			 if(_digitalPinMode[pin] == PinMode::DIGITAL_INPUT){
				 if(_digitalHistory[pin].size() > 0){
					 previous = _digitalHistory[pin].front();
				 }

				 mask = 1 << i;
				 _digitalHistory[pin].push_front((value & mask) >> i);

				 if((int)_digitalHistory[pin].size() > _digitalHistoryLength){
					 _digitalHistory[pin].pop_back();
				 }

				 // trigger an event if the pin has changed value
				 if(_digitalHistory[pin].front() != previous){
					 ofNotifyEvent(EDigitalPinChanged, pin, this);
				 }
			 }
		 }
		 break;

	 case 1: // pins 8-13 (in Firmata 2.3/Arduino 1.0, pins 14 and 15 are analog 0 and 1)
		 for(i = 0; i < port1Pins; ++i){
			 pin = i + 8;
			 previous = -1;
			 if(_digitalPinMode[pin] == PinMode::DIGITAL_INPUT){
				 if(_digitalHistory[pin].size() > 0){
					 previous = _digitalHistory[pin].front();
				 }

				 mask = 1 << i;
				 _digitalHistory[pin].push_front((value & mask) >> i);

				 if((int)_digitalHistory[pin].size() > _digitalHistoryLength){
					 _digitalHistory[pin].pop_back();
				 }

				 // trigger an event if the pin has changed value
				 if(_digitalHistory[pin].front() != previous){
					 ofNotifyEvent(EDigitalPinChanged, pin, this);
				 }
			 }
		 }
		 break;

	 case 2: // analog pins used as digital pins 16-21 (in Firmata 2.3/Arduino 1.0, digital pins 14 - 19)
		 for(i = 0; i < port2Pins; ++i){
			 //pin = i+analogOffset;
			 pin = i + 16;
			 previous = -1;
			 if(_digitalPinMode[pin] == PinMode::DIGITAL_INPUT){
				 if(_digitalHistory[pin].size() > 0){
					 previous = _digitalHistory[pin].front();
				 }

				 mask = 1 << i;
				 _digitalHistory[pin].push_front((value & mask) >> i);

				 if((int)_digitalHistory[pin].size() > _digitalHistoryLength){
					 _digitalHistory[pin].pop_back();
				 }

				 // trigger an event if the pin has changed value
				 if(_digitalHistory[pin].front() != previous){
					 ofNotifyEvent(EDigitalPinChanged, pin, this);
				 }
			 }
		 }
		 break;
	}
}

// port 0: pins 2-7  (0,1 are serial RX/TX, don't change their values)
// port 1: pins 8-13 (in Firmata 2.3/Arduino 1.0, pins 14 and 15 are analog pins 0 and 1 used as digital pins)
// port 2: pins 16-21 analog pins used as digital (in Firmata 2.3/Arduino 1.0, pins 14 - 19),
//         all analog reporting will be turned off if this is set to ARD_ON

void ofxFirmata::sendDigitalPortReporting(int port, int mode){
	sendByte((int)MessageType::REPORT_DIGITAL_PORT + port);
	sendByte(mode);
	_digitalPortReporting[port] = mode;
	int offset;

	if(_firmwareVersionSum >= FIRMWARE2_3){
		offset = 2;
	}else{
		offset = 0;
	}

	// for Firmata 2.3 and higher:
	if(port == 1 && mode == ARD_ON){
		for(int i = 0; i < 2; i++){
			_analogPinReporting[i] = ARD_OFF;
		}
	}

	// for Firmata 2.3 and all prior Firmata protocol versions:
	if(port == 2 && mode == ARD_ON){ // if reporting is turned on on port 2 then ofArduino on the Arduino disables all analog reporting

		for(int i = offset; i < ARD_TOTAL_ANALOG_PINS; i++){
			_analogPinReporting[i] = ARD_OFF;
		}
	}
}

void ofxFirmata::sendDigitalPinReporting(int pin, int mode){
	_digitalPinReporting[pin] = mode;
	int port1Offset;
	int port2Offset;

	// Firmata backwards compatibility mess
	if(_firmwareVersionSum >= FIRMWARE2_3){
		port1Offset = 15;
		port2Offset = 19;
	}else{
		port1Offset = 13;
		port2Offset = 21;
	}

	if(mode == ARD_ON){   // enable reporting for the port
		if(pin <= 7 && pin >= 2){
			sendDigitalPortReporting(0, ARD_ON);
		}
		// Firmata backwards compatibility mess
		if(pin <= port1Offset && pin >= 8){
			sendDigitalPortReporting(1, ARD_ON);
		}
		if(pin <= port2Offset && pin >= 16){
			sendDigitalPortReporting(2, ARD_ON);
		}
	}else if(mode == ARD_OFF){
		int i;
		bool send = true;
		if(pin <= 7 && pin >= 2){    // check if all pins on the port are off, if so set port reporting to off..
			for(i = 2; i < 8; ++i){
				if(_digitalPinReporting[i] == ARD_ON){
					send = false;
				}
			}
			if(send){
				sendDigitalPortReporting(0, ARD_OFF);
			}
		}
		// Firmata backwards compatibility mess
		if(pin <= port1Offset && pin >= 8){
			for(i = 8; i <= port1Offset; ++i){
				if(_digitalPinReporting[i] == ARD_ON){
					send = false;
				}
			}
			if(send){
				sendDigitalPortReporting(1, ARD_OFF);
			}
		}
		if(pin <= port2Offset && pin >= 16){
			for(i = 16; i <= port2Offset; ++i){
				if(_digitalPinReporting[i] == ARD_ON){
					send = false;
				}
			}
			if(send){
				sendDigitalPortReporting(2, ARD_OFF);
			}
		}
	}
}

void ofxFirmata::sendByte(unsigned char byte){
	//char msg[100];
	//sprintf(msg, "Sending Byte: %i", byte);
	//Logger::get("Application").information(msg);
	_port.writeByte(byte);
}

// in Firmata (and MIDI) data bytes are 7-bits. The 8th bit serves as a flag to mark a byte as either command or data.
// therefore you need two data bytes to send 8-bits (a char).
void ofxFirmata::sendValueAsTwo7bitBytes(int value){
	sendByte(value & 127); // LSB
	sendByte(value >> 7 & 127); // MSB
}

// SysEx data is sent as 8-bit bytes split into two 7-bit bytes, this function merges two 7-bit bytes back into one 8-bit byte.
int ofxFirmata::getValueFromTwo7bitBytes(unsigned char lsb, unsigned char msb){
	return (msb << 7) | lsb;
}

void ofxFirmata::sendServo(int pin, int value, bool force){
	// for firmata v2.2 and greater
	if(_firmwareVersionSum >= FIRMWARE2_2){
		if(_digitalPinMode[pin] == PinMode::SERVO && (_digitalPinValue[pin] != value || force)){
			sendByte((int)MessageType::ANALOG_IO_MESSAGE + pin);
			sendValueAsTwo7bitBytes(value);
			_digitalPinValue[pin] = value;
		}
	}
	// for versions prior to 2.2
	else{
		if(_digitalPinMode[pin] == PinMode::SERVO && (_servoValue[pin] != value || force)){
			sendByte(MessageType::START_SYSEX);
			sendByte(SYSEX_SERVO_WRITE);
			sendByte(pin);
			sendValueAsTwo7bitBytes(value);
			sendByte(MessageType::END_SYSEX);
			_servoValue[pin] = value;
		}
	}
}

// angle parameter is no longer supported. keeping for backwards compatibility
void ofxFirmata::sendServoAttach(int pin, int minPulse, int maxPulse, int angle){
	sendByte(MessageType::START_SYSEX);
	// for firmata v2.2 and greater
	if(_firmwareVersionSum >= FIRMWARE2_2){
		sendByte(MessageType::SERVO_CONFIG);
	}
	// for versions prior to 2.2
	else{
		sendByte(SYSEX_SERVO_ATTACH);
	}
	sendByte(pin);
	sendValueAsTwo7bitBytes(minPulse);
	sendValueAsTwo7bitBytes(maxPulse);
	sendByte(MessageType::END_SYSEX);
	_digitalPinMode[pin] = PinMode::SERVO;
}

// sendServoDetach depricated as of Firmata 2.2
void ofxFirmata::sendServoDetach(int pin){
	sendByte(MessageType::START_SYSEX);
	sendByte(SYSEX_SERVO_DETACH);
	sendByte(pin);
	sendByte(MessageType::END_SYSEX);
	_digitalPinMode[pin] = PinMode::DIGITAL_OUTPUT;
}

int ofxFirmata::getServo(int pin) const {
	if(_digitalPinMode[pin] == PinMode::SERVO){
		// for firmata v2.2 and greater
		if(_firmwareVersionSum >= FIRMWARE2_2){
			return _digitalPinValue[pin];
		}
		// for versions prior to 2.2
		else{
			return _servoValue[pin];
		}
	}else{
		return -1;
	}
}
