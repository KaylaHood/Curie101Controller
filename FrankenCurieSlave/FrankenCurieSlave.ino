#include <CurieIMU.h>
#include <CurieBLE.h>
#include <math.h>
#include <limits.h>
#include <cstdint>
#include "FrankenCurieSlave.h"

enum Opcodes { Normal, ZeroMotion, Calibration };

bool serialIsConnected = false;

char calibrationMsg = 'c';

uint64_t microsNow;
uint64_t zeroMotionDetectedMicros = 100000000; // large to prevent unintentional trigger
uint64_t zMT = zeroMotionDetectedMicros;
uint64_t interval = 1500000;

float zeroMotionThreshold = 1.95f;
float zeroMotionDuration = 2.0f;

int16_t gyroRange = 500;
int16_t accelRange = 2;
int16_t sampleRate = 1600;
int16_t gyroSensitivity = 1;

bool calibrateAgain = false;

Message slaveMsg = Message(8);

void setup() {
	Serial1.begin(38400);
	while (!Serial1); 

	CurieIMU.begin();

	CurieIMU.attachInterrupt(eventCallback);
	CurieIMU.setDetectionThreshold(CURIE_IMU_ZERO_MOTION, zeroMotionThreshold);
	CurieIMU.setDetectionDuration(CURIE_IMU_ZERO_MOTION, zeroMotionDuration);
	CurieIMU.interrupts(CURIE_IMU_ZERO_MOTION);

	CurieIMU.setAccelerometerRange(accelRange);
	CurieIMU.setAccelerometerRate(sampleRate);
	CurieIMU.BMI160Class::setAccelDLPFMode(BMI160_DLPF_MODE_OSR4);
}

void loop() {
	if (Serial1.available() > 0) {
		char cmd = Serial1.read();
		Serial1.flush();
		if (serialIsConnected) {
			if (cmd == calibrationMsg) {
				zMT = zeroMotionDetectedMicros;
				microsNow = micros();
				calibrate();
			}
			else if (cmd == 'd') {
				serialIsConnected = false;
			}
			else if (cmd == 'u') {
				updateValues(Normal);
			}
			else if (calibrateAgain) {
				calibrateAgain = false;
				zMT = zeroMotionDetectedMicros;
				microsNow = micros();
				calibrate();
			}
		}
		else if (cmd == 'y') {
			serialIsConnected = true;
		}
	}
	else if (serialIsConnected && calibrateAgain) {
		calibrateAgain = false;
		zMT = zeroMotionDetectedMicros;
		microsNow = micros();
		calibrate();
	}
}

void updateValues(int16_t opcode) {
	int rawAx32, rawAy32, rawAz32;
	int16_t rawAx16 = 0, rawAy16 = 0, rawAz16 = 0;

	CurieIMU.readAccelerometer(rawAx32, rawAy32, rawAz32);

	rawAx16 = rawAx32;
	rawAy16 = rawAy32;
	rawAz16 = rawAz32;

	if (isZeroMotion(microsNow, zMT) && opcode != Calibration) {
		opcode = ZeroMotion;
	}

	slaveMsg.setValue<int16_t>(opcode, 0);
	slaveMsg.setValue<int16_t>(rawAx16, 2);
	slaveMsg.setValue<int16_t>(rawAy16, 4);
	slaveMsg.setValue<int16_t>(rawAz16, 6);

	if (Serial1.peek() == calibrationMsg && opcode != Calibration) {
		Serial1.flush();
		return;
	}
	slaveMsg.send();
	Serial1.flush();
}

void calibrate() {
	if (isZeroMotion(microsNow, zMT)) {
		CurieIMU.autoCalibrateXAccelOffset(0);
		CurieIMU.autoCalibrateYAccelOffset(0);
		CurieIMU.autoCalibrateZAccelOffset(1);

		updateValues(Calibration);
	}
	else {
		calibrateAgain = true;
	}
}

static void eventCallback(void) {
	if (CurieIMU.getInterruptStatus(CURIE_IMU_ZERO_MOTION)) {
		zeroMotionDetectedMicros = micros();
	}
}

bool isZeroMotion(uint64_t time, uint64_t zeroMotionTime) {
	return ((time - zeroMotionTime) < interval);
}

Message::Message() {
	this->message = nullptr;
	size = 0;
}

Message::Message(int16_t s) :size(s) {
	this->message = new uint8_t[s];
}

Message& Message::operator=(const Message& sm) {
	this->size = sm.size;
	this->message = new uint8_t[this->size];
	memcpy(this->message, sm.message, this->size);
	return *this;
}

Message::~Message() {
	delete[] this->message;
}

template <typename T>
void Message::setValue(const T& val, int pos) {
	memcpy((this->message + pos), &val, sizeof(T));
}

template <typename T>
T Message::getValue(int pos) const {
	if ((pos + sizeof(T)) <= this->size) {
		T value{};
		memcpy(&value, (this->message + pos), sizeof(T));
		return value;
	}
	return T{};
}

void Message::send() const {
	for (int i = 0; i < this->size; i++) {
		Serial1.write(*(this->message + i));
	}
}

void Message::debugPrintMaster() const {}

void Message::debugPrintSlave() const {}