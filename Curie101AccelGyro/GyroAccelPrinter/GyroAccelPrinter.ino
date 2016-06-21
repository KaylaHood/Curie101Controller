#include <CurieIMU.h>
#include <math.h>
#include <limits.h>
#include <cstdint>
#include "GyroAccelPrinter.h"

// program time in microseconds of last "update" call
unsigned long microsNow;
// program time in microseconds of last zero motion interrupt
unsigned long zeroMotionDetectedMicros = 100000000; // large to prevent unintentional trigger
// interval in microseconds for zero motion detection in update function
// if abs(zeroMotionDetectedMicros - microsNow) < interval, then zero motion is reported
unsigned long interval = 1280000;

// the threshold in micro Gs of the zero motion detection hardware
float zeroMotionThreshold = 1500.0f;
// the duration in seconds of zero motion detection 
// needed for the Curie to fire an interrupt 
float zeroMotionDuration = 5.0f;

// the range in degrees/second of the gyroscope
int gyroRange = 500;
// the range in Gs of the accelerometer
int accelRange = 2;
// the sample rate in Hz of gyroscope and accelerometer
int sampleRate = 25;
// the sensitivity of the gyroscope (divisor of raw gyroscope values)
int gyroSensitivity = 80;

// program time in microseconds of previous update
unsigned long microsPrevious;
// "tick" rate of update
// "updateValues" function restricts occurrence
// of updates to match the interval defined by this variable
unsigned long microsPerReading = 1000000 / sampleRate;
// last time calibrated in microseconds
unsigned long calibrationTime;
// if calibration fails, set this to true so that next loop knows to attempt calibration again
bool calibrateAgain = false;

void setup() {
	Serial.begin(9600); // initialize Serial communication
	while (!Serial);    // wait for the serial port to open

	// initialize device
	CurieIMU.begin();

	// Set up zero motion interrupts
	CurieIMU.attachInterrupt(eventCallback);
	CurieIMU.setDetectionThreshold(CURIE_IMU_ZERO_MOTION, zeroMotionThreshold);
	CurieIMU.setDetectionDuration(CURIE_IMU_ZERO_MOTION, zeroMotionDuration);
	CurieIMU.interrupts(CURIE_IMU_ZERO_MOTION);

	// initialize time
	microsPrevious = micros();

	// Set the accelerometer range
	CurieIMU.setGyroRange(gyroRange);
	CurieIMU.setGyroRate(sampleRate);
	CurieIMU.setAccelerometerRange(accelRange);
	CurieIMU.setAccelerometerRate(sampleRate);

	// note - do not calibrate yet
	// the Curie board might not be flat and might not be stationary
	// and the user may not want to calibrate immediately
}

void loop() {
	if (Serial.available() > 0) {
		char command = char(Serial.read());
		if (command == 'c') {
			microsNow = micros();
			calibrate();
		}
	}
	else if (calibrateAgain) {
		calibrateAgain = false;
		microsNow = micros();
		calibrate();
	}
	else updateValues(1);
}

// updateValues reads the sensors and then prints data to the SerialPort buffer.
// The grammar of the serial print is as such:
// ***
// "opcode,timestamp,acc x-value,acc y-value,acc z-value,gyro x-value,gyro y-value,gyro z-value;"
// ***
// The x-value, y-value, and z-value are integer values (such as "20" or "9")
// The first set are from the accelerometer and the second set are from the gyroscope.
//
// The opcode is an int used to identify which hardware the values
// are from, and to determine the state of the Curie at the
// time of gathering those values.
// These are the opcodes that this code uses:
//		0 = values directly post-calibration (board is flat and stationary)
//		1 = values a while after calibration (device in use)
//		2 = values during a zero-motion event
//
// The timestamp is the time (microsNow) in microseconds. This will overflow and
// go back to zero after about 70 minutes. Consider keeping track of overflows in
// whatever program is recieving the data from the Curie.
//
// If the opcode is 0 (calibration), then the gyro x-value will be the Accelerometer Range,
// and the gyro y-value will be the Gyroscope Range. This is used to tell the receiving program
// what values to use when converting the raw values into useful units.
//
void updateValues(int opcode) {
	int rawAx, rawAy, rawAz;
	int rawGx, rawGy, rawGz;

	microsNow = micros();
	// detect time overflow
	if (microsNow < microsPrevious) {
		// deal with time overflow
		// use 64-bit integer types to allow signed values with MIN and MAX
		// values that are identical to those of unsigned long ints
		int64_t mP = microsPrevious;
		int64_t mN = microsNow;
		mP -= ULONG_MAX;
		int64_t diff = mN - mP;
		// block until correct time
		while (diff < microsPerReading) {
			microsNow = micros();
			mN = microsNow;
			diff = mN - mP;
		}
	}
	else {
		// block until correct time
		while (microsNow - microsPrevious < microsPerReading) {
			microsNow = micros();
		}
	}

	// read raw measurements from device
	CurieIMU.readMotionSensor(
		rawAx, 
		rawAy, 
		rawAz, 
		rawGx, 
		rawGy, 
		rawGz
	);

	// if zero motion event is occurring and the opcode is not 0
	// (meaning this is not a calibration update), then set opcode
	// to 2, which is the zero-motion opcode.
	if(isZeroMotion(microsNow) && opcode != 0) {
		opcode = 2;
	}

	// print raw values from accelerometer and gyroscope, along with opcode and timestamp
	String values = "";
	values += String(opcode);
	values += ",";
	values += String(microsNow);
	values += ",";
	values += String(rawAx);
	values += ",";
	values += String(rawAy);
	values += ",";
	values += String(rawAz);
	values += ",";
	if (opcode == 0) {
		values += String(CurieIMU.getAccelerometerRange());
		values += ",";
		values += String(CurieIMU.getGyroRange());
	}
	else {
		values += String(rawGx / gyroSensitivity);
		values += ",";
		values += String(rawGy / gyroSensitivity);
	}
	values += ",";
	values += String(rawGz / gyroSensitivity);
	values += ";";

	Serial.print(values);

	// update the time keeping variable
	microsPrevious = microsNow;
}

void calibrate() {
	if (isZeroMotion(microsNow)) {
		// calibrate gyroscope
		CurieIMU.autoCalibrateGyroOffset();
		
		// calibrate accelerometer
		CurieIMU.autoCalibrateXAccelOffset(0);
		CurieIMU.autoCalibrateYAccelOffset(0);
		CurieIMU.autoCalibrateZAccelOffset(1);

		updateValues(0);

		calibrationTime = microsPrevious;
	}
	else {
		// zero motion not detected, try again on next loop
		calibrateAgain = true;
	}
}

static void eventCallback(void) {
	if (CurieIMU.getInterruptStatus(CURIE_IMU_ZERO_MOTION)) {
		zeroMotionDetectedMicros = micros();
		//Serial.print("interrupt fired");
		//Serial.println();
	}
}

bool isZeroMotion(unsigned long time) {
	// return true if zero motion was detected within
	// [interval] microseconds from the supplied time
	// return false otherwise
	return (abs(time - zeroMotionDetectedMicros) < interval);
}