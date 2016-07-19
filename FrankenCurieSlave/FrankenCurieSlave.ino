#include <CurieIMU.h>
#include <CurieBLE.h>
#include <math.h>
#include <limits.h>
#include <cstdint>
#include "FrankenCurieSlave.h"

// -----------------------------------
// Opcode Enum for Serial Port Opcodes
// -----------------------------------
enum Opcodes { Normal, ZeroMotion, Calibration };

// boolean for indicating computer's presence on SerialPort
bool serialIsConnected = false;

char calibrationMsg = 'c';

// -----------------------------------
// Gyroscope / Accelerometer variables
// -----------------------------------
// program time in microseconds of last "update" call
uint64_t microsNow;
// program time in microseconds of *most recent* zero motion interrupt
uint64_t zeroMotionDetectedMicros = 100000000; // large to prevent unintentional trigger
// zMT is the time of the zero motion interrupt just before the current update
// *** this variable is set right before setting "microsNow" so that zero motion interrupts
//     between setting "microsNow" and testing for zero motion don't cause errors within
//     the "isZeroMotion" function. We need to know the zero motion event that was detected
//     *before* the time "microsNow".
uint64_t zMT = zeroMotionDetectedMicros;
// interval in microseconds for zero motion detection in update function
// *** if abs(zeroMotionDetectedMicros - microsNow) < interval, then zero motion is reported
uint64_t interval = 1500000;

// the threshold in micro-G's of the zero motion detection hardware
// Accel Range | Min Zero Motion Threshold | Max Zero Motion Threshold
// 2 G         | 1.95 mG                   | 3.91 mG
// 4 G         | 3.92 mG                   | 7.81 mG
// 8 G         | 7.82 mG                   | 15.63 mG
// 16 G        | 15.64 mG                  | 31.25 mG
float zeroMotionThreshold = 1.95f;
// the duration in seconds of zero motion detection 
// needed for the Curie to fire an interrupt 
float zeroMotionDuration = 2.0f;

// the range in degrees/second of the gyroscope
int16_t gyroRange = 500;
// the range in Gs of the accelerometer
int16_t accelRange = 2;
// the sample rate in Hz of gyroscope and accelerometer
int16_t sampleRate = 1600;
// the sensitivity of the gyroscope (divisor of raw gyroscope values)
// *** larger number = less sensitive reading
int16_t gyroSensitivity = 1;

// if calibration fails, set this to true so that next loop knows to attempt calibration again
bool calibrateAgain = false;

// message holder for communication using a byte array
Message slaveMsg = Message(8);

void setup() {
	Serial1.begin(38400);
	// wait for the serial ports to open
	while (!Serial1); 

	// initialize device
	CurieIMU.begin();

	// Set up zero motion interrupts
	CurieIMU.attachInterrupt(eventCallback);
	CurieIMU.setDetectionThreshold(CURIE_IMU_ZERO_MOTION, zeroMotionThreshold);
	CurieIMU.setDetectionDuration(CURIE_IMU_ZERO_MOTION, zeroMotionDuration);
	CurieIMU.interrupts(CURIE_IMU_ZERO_MOTION);

	// Set the accelerometer range and rate
	CurieIMU.setAccelerometerRange(accelRange);
	CurieIMU.setAccelerometerRate(sampleRate);
	// Configure digital low-pass filters on BMI160 IMU chip 
	// *** (set to most selective filter setting -- slows down actual rate of 
	//     reading from sensor but weeds out faulty data better)
	CurieIMU.BMI160Class::setAccelDLPFMode(BMI160_DLPF_MODE_OSR4);
}

void loop() {
	if (Serial1.available() > 0) {
		// read request from the serial port
		char cmd = Serial1.read();
		Serial1.flush();
		if (serialIsConnected) {
			if (cmd == calibrationMsg) {
				// Master is requesting calibration
				// save last zero motion time before setting "microsNow"
				zMT = zeroMotionDetectedMicros;
				microsNow = micros();
				calibrate();
			}
			else if (cmd == 'd') {
				// Computer has disconnected
				// set connection boolean to false
				serialIsConnected = false;
			}
			else if (cmd == 'u') {
				// Master requested updated values 
				updateValues(Normal);
			}
			else if (calibrateAgain) {
				calibrateAgain = false;
				// save last zero motion time before setting "microsNow"
				zMT = zeroMotionDetectedMicros;
				microsNow = micros();
				calibrate();
			}
		}
		else if (cmd == 'y') {
			// Computer has requested communication to begin
			// set connection boolean to true
			serialIsConnected = true;
		}
	}
	else if (serialIsConnected && calibrateAgain) {
		calibrateAgain = false;
		// save last zero motion time before setting "microsNow"
		zMT = zeroMotionDetectedMicros;
		microsNow = micros();
		calibrate();
	}
}

void updateValues(int16_t opcode) {
	// updateValues reads the sensors and then prints data to the SerialPort buffer.
	//
	// The timestamp is the time (microsNow) in microseconds. This will overflow and
	// go back to zero after about 18000 years. We will not manage overflows.
	//
	// If the opcode is for calibration, then the gyro x-value will be the Accelerometer Range,
	// and the gyro y-value will be the Gyroscope Range. This is used to tell the receiving program
	// what values to use when converting the raw values into useful units.
	//
	int rawAx32, rawAy32, rawAz32;
	int16_t rawAx16 = 0, rawAy16 = 0, rawAz16 = 0;

	// update values to sensor readings if opcode is not for calibration
	if (opcode != Calibration) {
		// read raw measurements from device 
		// *** (IMU library returns 32-bit ints for IMU data, even though hardware data is 16-bit ints)
		CurieIMU.readAccelerometer(rawAx32, rawAy32, rawAz32);

		// do conversion to 16 bits 
		// *** (Curie's hardware IMU device returns 16-bit values for data, no information is lost here)
		rawAx16 = rawAx32;
		rawAy16 = rawAy32;
		rawAz16 = rawAz32;
	}

	// if zero motion event is occurring and the opcode is not 0
	// (meaning this is not a calibration update), then set opcode
	// to 2, which is the zero-motion opcode.
	if (isZeroMotion(microsNow, zMT) && opcode != Calibration) {
		opcode = ZeroMotion;
	}

	// set up values in slaveMsg
	slaveMsg.setValue<int16_t>(opcode, 0);
	slaveMsg.setValue<int16_t>(rawAx16, 2);
	slaveMsg.setValue<int16_t>(rawAy16, 4);
	slaveMsg.setValue<int16_t>(rawAz16, 6);

	// before sending data, check for calibration request
	// *** only stop updating if not already calibrating
	if (Serial1.peek() == calibrationMsg && opcode != Calibration) {
		// calibration request received! Abort update
		Serial1.flush();
		return;
	}
	// write byte values to Serial Port
	slaveMsg.send();
	Serial1.flush();
}

void calibrate() {
	if (isZeroMotion(microsNow, zMT)) {
		// calibrate accelerometer
		CurieIMU.autoCalibrateXAccelOffset(0);
		CurieIMU.autoCalibrateYAccelOffset(0);
		CurieIMU.autoCalibrateZAccelOffset(1);

		updateValues(Calibration);
	}
	else {
		// zero motion not detected, try again on next loop
		calibrateAgain = true;
	}
}

static void eventCallback(void) {
	if (CurieIMU.getInterruptStatus(CURIE_IMU_ZERO_MOTION)) {
		zeroMotionDetectedMicros = micros();
	}
}

bool isZeroMotion(uint64_t time, uint64_t zeroMotionTime) {
	// return true if zero motion was detected within
	// [interval] microseconds from the supplied time
	// return false otherwise
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

// destructor
Message::~Message() {
	delete[] this->message;
}

// byte setting functions
template <typename T>
void Message::setValue(const T& val, int pos) {
	// insert bits from "val" into byte index "pos" in message
	memcpy((this->message + pos), &val, sizeof(T));
}

// retrieval functions
template <typename T>
T Message::getValue(int pos) const {
	// get a pointer to type T (use reinterpret_cast to force the type)
	if ((pos + sizeof(T)) <= this->size) {
		T value{};
		memcpy(&value, (this->message + pos), sizeof(T));
		return value;
	}
	return T{};
}

// write values to serial port
void Message::send() const {
	for (int i = 0; i < this->size; i++) {
		Serial1.write(*(this->message + i));
	}
}

void Message::debugPrintMaster() const {}

void Message::debugPrintSlave() const {}

void Message::debugPrintBLE() const {}