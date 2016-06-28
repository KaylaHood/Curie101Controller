#include <CurieIMU.h>
#include <CurieBLE.h>
#include <math.h>
#include <limits.h>
#include <cstdint>
#include "GyroAccelPrinter.h"

// -----------------------------------
// BLE variables
// -----------------------------------
BLEPeripheral blePeripheral;
BLEService IMUService("19B10010-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic IMUCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLENotify | BLERead | BLEWrite , 17);
bool bleIsConnected = false;

// -----------------------------------
// Gyroscope / Accelerometer variables
// -----------------------------------
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
int sampleRate = 500;
// the sensitivity of the gyroscope (divisor of raw gyroscope values)
int gyroSensitivity = 1;

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

	// Set the local name that the peripheral advertises
	blePeripheral.setLocalName("CurieIMU");
	blePeripheral.setAdvertisedServiceUuid(IMUService.uuid());

	// add service and characteristics
	blePeripheral.addAttribute(IMUService);
	blePeripheral.addAttribute(IMUCharacteristic);

	unsigned char initialValue[17] = { '0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0' };
	IMUCharacteristic.setValue(initialValue ,17);

	// advertise the service
	blePeripheral.begin();

}

void loop() {
	// listen for BLE peripherals to connect
	BLECentral bleCentral = blePeripheral.central();

	// set toggle for BLE connection
	if (bleCentral) {
		bleIsConnected = true;
	}
	else {
		bleIsConnected = false;
	}

	// detect request for calibration from serial port or BLE central
	if (Serial.available() > 0 || IMUCharacteristic.written()) {
		// Serial is only written to when Curie is asked to calibrate. 
		// Check to see if the command is 'c' to prevent accidental calibration
		char command = char(Serial.read());
		// BLEcommand is the opcode - if the opcode (first char of "value") is the char '3', then calibrate
		unsigned char BLEcommand = IMUCharacteristic.value()[0];
		if (command == 'c' || BLEcommand == '3') {
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
	rawGx /= gyroSensitivity;
	rawGy /= gyroSensitivity;
	rawGz /= gyroSensitivity;

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
		values += String(rawGx);
		values += ",";
		values += String(rawGy);
	}
	values += ",";
	values += String(rawGz);
	values += ";";

	Serial.print(values);

	// check to see if a central is connected to peripheral
	if (bleIsConnected) {
		// construct BLE string to fit into 17 bytes
		unsigned char BLEvalues[17];
		BLEvalues[0] = (unsigned char)opcode;
		// convert timestamp (4 byte unsigned long) into 4 chars with identical bit structure
		BLEvalues[1] = (unsigned char)(microsNow >> 24);
		BLEvalues[2] = (unsigned char)((microsNow << 8) >> 24);
		BLEvalues[3] = (unsigned char)((microsNow << 16) >> 24);
		BLEvalues[4] = (unsigned char)((microsNow << 24) >> 24);
		// convert accelerometer values to chars
		BLEvalues[5] = (unsigned char)(rawAx >> 8);
		BLEvalues[6] = (unsigned char)((rawAx << 8) >> 8);
		BLEvalues[7] = (unsigned char)(rawAy >> 8);
		BLEvalues[8] = (unsigned char)((rawAy << 8) >> 8);
		BLEvalues[9] = (unsigned char)(rawAz >> 8);
		BLEvalues[10] = (unsigned char)((rawAz << 8) >> 8);
		// convert gyroscope values to chars
		BLEvalues[11] = (unsigned char)(rawGx >> 8);
		BLEvalues[12] = (unsigned char)((rawGx << 8) >> 8);
		BLEvalues[13] = (unsigned char)(rawGy >> 8);
		BLEvalues[14] = (unsigned char)((rawGy << 8) >> 8);
		BLEvalues[15] = (unsigned char)(rawGz >> 8);
		BLEvalues[16] = (unsigned char)((rawGz << 8) >> 8);

		// update characteristic value
		IMUCharacteristic.setValue(BLEvalues, 17);
	}

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