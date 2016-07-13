#include <CurieIMU.h>
#include <CurieBLE.h>
#include <math.h>
#include <limits.h>
#include <cstdint>
#include "GyroAccelPrinter.h"

#ifdef SERIAL_ENABLED
// -----------------------------------
// Opcode Enum for Serial Port Opcodes
// -----------------------------------
enum Opcodes {Normal, ZeroMotion, Calibration};

// boolean for indicating computer's presence on SerialPort
bool serialIsConnected = false;

#endif

#ifdef BLE_ENABLED
// -----------------------------------
// BLE variables
// -----------------------------------
int16_t characteristicSize = 20;
BLEPeripheral blePeripheral;
BLEService IMUService("19B10010-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic IMUCalibrationCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLENotify | BLERead | BLEWrite , 20);
BLECharacteristic IMUMotionCharacteristic("19B10012-E8F2-537E-4F6C-D104768A1214", BLENotify | BLERead | BLEWrite , 20);
BLECharacteristic IMUZeroMotionCharacteristic("19B10013-E8F2-537E-4F6C-D104768A1214", BLENotify | BLERead | BLEWrite , 20);
BLECharacteristic IMURequestCharacteristic("19B10014-E8F2-537E-4F6C-D104768A1214", BLENotify | BLERead | BLEWrite , 20);
// boolean for indicating computer's presence on BLE
bool bleIsConnected = false;
#endif

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

// program time in microseconds of previous update
uint64_t microsPrevious;
// microsPerUpdate is the time in microseconds of each update.
// *** "updateValues" function restricts occurrence
//     of updates to match the interval defined by this variable
// *** microsPerUpdate must be greater than or equal to :
//     (microsPerSecond) / (sampleRate / DLPF Sample Rate)
uint64_t microsPerUpdate = 1000000 / (sampleRate / 4);
// last time calibrated in microseconds
uint64_t calibrationTime;
// if calibration fails, set this to true so that next loop knows to attempt calibration again
bool calibrateAgain = false;

// message holder for communication using a byte array
Message statement;

void setup() {
	#ifdef SERIAL_ENABLED
	Serial.begin(9600); // initialize Serial communication
	while (!Serial);    // wait for the serial port to open
	// -------------------------
	// Set Up Serial Statement Variable
	// -------------------------
	statement = Message(22);
	#endif

	// initialize device
	CurieIMU.begin();

	// Set up zero motion interrupts
	CurieIMU.attachInterrupt(eventCallback);
	CurieIMU.setDetectionThreshold(CURIE_IMU_ZERO_MOTION,zeroMotionThreshold);
	CurieIMU.setDetectionDuration(CURIE_IMU_ZERO_MOTION,zeroMotionDuration);
	CurieIMU.interrupts(CURIE_IMU_ZERO_MOTION);

	// initialize time
	microsPrevious = micros();

	// Set the accelerometer range
	CurieIMU.setGyroRange(gyroRange);
	CurieIMU.setGyroRate(sampleRate);
	CurieIMU.setAccelerometerRange(accelRange);
	CurieIMU.setAccelerometerRate(sampleRate);
	// Configure digital low-pass filters on BMI160 IMU chip 
	// *** (set to most selective filter setting -- slows down actual rate of 
	//     reading from sensor but weeds out faulty data better)
	CurieIMU.BMI160Class::setGyroDLPFMode(BMI160_DLPF_MODE_OSR4);
	CurieIMU.BMI160Class::setAccelDLPFMode(BMI160_DLPF_MODE_OSR4);

	#ifdef BLE_ENABLED
	// -----------------------------------
	// BLE set-up
	// -----------------------------------

	// Set the local name that the peripheral advertises
	blePeripheral.setLocalName("CurieIMU");
	blePeripheral.setAdvertisedServiceUuid(IMUService.uuid());

	// add service and characteristics
	blePeripheral.addAttribute(IMUService);
	blePeripheral.addAttribute(IMUCalibrationCharacteristic);
	blePeripheral.addAttribute(IMUMotionCharacteristic);
	blePeripheral.addAttribute(IMUZeroMotionCharacteristic);
	blePeripheral.addAttribute(IMURequestCharacteristic);

	unsigned char initialValue[characteristicSize];
	IMUCalibrationCharacteristic.setValue(initialValue, 20);
	IMUMotionCharacteristic.setValue(initialValue, 20);
	IMUZeroMotionCharacteristic.setValue(initialValue, 20);
	IMURequestCharacteristic.setValue(initialValue, 20);

	// advertise the service
	blePeripheral.begin();
	#endif
}

void loop() {
	#ifdef CURIE_SPEED_DEBUG
	Serial.print("starting loop time: ");
	Serial.print(micros());
	Serial.println();
	#endif

	#ifdef BLE_ENABLED
	// listen for BLE peripherals to connect
	BLECentral bleCentral = blePeripheral.central();
	// set toggle for BLE connection
	if (bleCentral) {
		bleIsConnected = true;
		if (IMURequestCharacteristic.written()) {
			#if defined CURIE_CALIBRATION_DEBUG || defined CURIE_BLE_DEBUG
			Serial.print("BLE recieved request.");
			Serial.println();
			#endif	
			// save last zero motion time before setting "microsNow"
			zMT = zeroMotionDetectedMicros;
			microsNow = micros();
			calibrate();
		}
		else if (calibrateAgain) {
			calibrateAgain = false;
			// save last zero motion time before setting "microsNow"
			zMT = zeroMotionDetectedMicros;
			microsNow = micros();
			calibrate();
		}
		else updateValues(Normal);
	}
	else {
		bleIsConnected = false;
	}
	#endif

	#ifdef SERIAL_ENABLED
	if (serialIsConnected) {
		// detect commands from serial port
		if (Serial.available() > 0) {
			#if defined CURIE_CALIBRATION_DEBUG || defined CURIE_SERIAL_DEBUG
			Serial.print("Serial port was written to externally.");
			Serial.println();
			#endif	
			// read request from the serial port
			char cmd = Serial.read();
			//
			// The Serial Port is written to when Curie is asked to calibrate. 
			//
			if (cmd == 'c') {
				// Curie is requesting calibration
				// save last zero motion time before setting "microsNow"
				zMT = zeroMotionDetectedMicros;
				microsNow = micros();
				calibrate();
			}
			else if (cmd == 'd') {
				// Curie has disconnected
				// set connection boolean to false
				serialIsConnected = false;
			}
			else {
				updateValues(Normal);
			}
		}
		else {
			updateValues(Normal);
		}
	}
	else if(Serial.available() > 0){
		// detect connection message ("y")
		char cmd = Serial.read();
		if (cmd == 'y') {
			// set connection boolean to true
			serialIsConnected = true;
		}
	}
	#endif

	#ifdef CURIE_SPEED_DEBUG
	Serial.print("ending loop time: ");
	Serial.print(micros());
	Serial.println();
	#endif
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
	#ifdef CURIE_SPEED_DEBUG
	Serial.print("update start time: ");
	Serial.print(micros());
	Serial.println();
	#endif
	int rawAx32, rawAy32, rawAz32;
	int rawGx32, rawGy32, rawGz32;
	int16_t rawAx16 = 0, rawAy16 = 0, rawAz16 = 0;
	int16_t rawGx16 = 0, rawGy16 = 0, rawGz16 = 0;

	// block until correct time
	while ((micros() - microsPrevious) < microsPerUpdate) {
		#ifdef CURIE_SPEED_DEBUG
		Serial.print("waiting...");
		Serial.println();
		#endif
	}
	// save most recent zero motion detected time, because the zero motion interrupt *could*
	// fire inbetween setting "microsNow" and testing for zero motion, which would result
	// in the function "isZeroMotion" returning false when zero motion is happening.
	zMT = zeroMotionDetectedMicros;
	microsNow = micros();

	// update values to sensor readings if opcode is not for calibration
	if (opcode != Calibration) {
		// read raw measurements from device (IMU library returns standard 32-bit ints for IMU data)
		CurieIMU.readMotionSensor(
			rawAx32,
			rawAy32,
			rawAz32,
			rawGx32,
			rawGy32,
			rawGz32
		);
		rawGx32 /= gyroSensitivity;
		rawGy32 /= gyroSensitivity;
		rawGz32 /= gyroSensitivity;

		// do conversion to 16 bits 
		// (Curie's IMU returns 16-bit values for data, no information is lost here)
		rawAx16 = rawAx32;
		rawAy16 = rawAy32;
		rawAz16 = rawAz32;
		rawGx16 = rawGx32;
		rawGy16 = rawGy32;
		rawGz16 = rawGz32;
	}

	// if zero motion event is occurring and the opcode is not 0
	// (meaning this is not a calibration update), then set opcode
	// to 2, which is the zero-motion opcode.
	if(isZeroMotion(microsNow,zMT) && opcode != Calibration) {
		opcode = ZeroMotion;
	}

	// else, if this is calibration, set rawGx to accelerometer range, and rawGy to gyroscope range
	else if (opcode == Calibration) {
		rawGx16 = accelRange;
		rawGy16 = gyroRange;
	}

	// set up values in statement
	statement.setValue<int16_t>(opcode, 0);
	statement.setValue<uint64_t>(microsNow, 2);
	statement.setValue<int16_t>(rawAx16, 10);
	statement.setValue<int16_t>(rawAy16, 12);
	statement.setValue<int16_t>(rawAz16, 14);
	statement.setValue<int16_t>(rawGx16, 16);
	statement.setValue<int16_t>(rawGy16, 18);
	statement.setValue<int16_t>(rawGz16, 20);

	// -----------------------------------
	// BLE Characteristic and Serial Port Update
	// -----------------------------------
	// syntax of 22-byte characteristic:
	// -----
	// [2 bytes for opcode][8 bytes for 8-byte timestamp][six bytes for each 2-byte accelerometer value][six bytes for each 2-byte gyroscope value]
	// -----
	// "opcode" = binary representation of either '0','1', or '2'
	//     '0' = normal values
	//     '1' = zero motion detected
	//     '2' = calibration
	// "timestamp" is a 8-byte "uint64_t" type, and the bytes have the same binary
	//     representation as the original value.
	// the sets of data for the accelerometer and gyroscope have the same binary
	//     equivalence as the timestamp chars.
	// -----
	// the opcode, timestamp and data sets must be transformed byte-by-byte using bitwise 
	// functions to convert them back into number values.

	#ifdef BLE_ENABLED
	// check for connected central
	if (bleIsConnected) {
		// construct unsigned char array of values because the BLE "setValue" function 
		// requires an unsigned char array
		unsigned char ucvalues[characteristicSize];
		// use memcpy because we don't want the byte data to be interpreted at all 
		// into characters. We just want the binary representation of those bytes.
		// Get pointer to the statement at index 2 because we don't want the opcode bytes.
		// (we can only fit 20 bytes into the BLE characteristic)
		memcpy(ucvalues, (statement.message + 2), characteristicSize);

		// update characteristic value
		if (opcode == Calibration) {
			IMUCalibrationCharacteristic.setValue(ucvalues, characteristicSize);
		}
		else if (opcode == Normal) {
			IMUMotionCharacteristic.setValue(ucvalues, characteristicSize);
		}
		else if (opcode == ZeroMotion) {
			IMUZeroMotionCharacteristic.setValue(ucvalues, characteristicSize);
		}
	}
	#endif

	#ifndef CURIE_SERIAL_DEBUG
	// write byte values to Serial Port
	statement.send();
	#else
	Serial.print("Actual values (decimal): ");
	Serial.print(opcode);
	Serial.print("|");
	Serial.print(microsNow);
	Serial.print("|");
	Serial.print(rawAx16);
	Serial.print("|");
	Serial.print(rawAy16);
	Serial.print("|");
	Serial.print(rawAz16);
	Serial.print("|");
	Serial.print(rawGx16);
	Serial.print("|");
	Serial.print(rawGy16);
	Serial.print("|");
	Serial.print(rawGz16);
	Serial.println();
	// set-up serial port statement
	statement.debugPrint();
	#endif

	// update the time keeping variable
	microsPrevious = microsNow;
	#ifdef CURIE_SPEED_DEBUG
	Serial.print("update end time: ");
	Serial.print(micros());
	Serial.println();
	#endif
	#ifdef SERIAL_ENABLED
	Serial.flush();
	#endif
}

void calibrate() {
	if (isZeroMotion(microsNow,zMT)) {
		// calibrate gyroscope
		CurieIMU.autoCalibrateGyroOffset();
		
		// calibrate accelerometer
		CurieIMU.autoCalibrateXAccelOffset(0);
		CurieIMU.autoCalibrateYAccelOffset(0);
		CurieIMU.autoCalibrateZAccelOffset(1);

		updateValues(Calibration);

		calibrationTime = microsPrevious;
	}
	else {
		// zero motion not detected, try again on next loop
		calibrateAgain = true;
		updateValues(Normal);
	}
}

static void eventCallback(void) {
	if (CurieIMU.getInterruptStatus(CURIE_IMU_ZERO_MOTION)) {
		zeroMotionDetectedMicros = micros();
		#ifdef CURIE_INTERRUPT_DEBUG
		Serial.println();
		Serial.print("Zero motion interrupt fired.");
		Serial.println();
		#endif
	}
}

bool isZeroMotion(uint64_t time, uint64_t zeroMotionTime) {
	// return true if zero motion was detected within
	// [interval] microseconds from the supplied time
	// return false otherwise
	#ifdef CURIE_NOMO_DEBUG
	Serial.print("Update time: ");
	Serial.print(time);
	Serial.print(", zMT: ");
	Serial.print(zeroMotionTime);
	Serial.println();
	#endif	
	return ((time - zeroMotionTime) < interval);
}

// -----------------------------------------
// Message struct function definitions
// -----------------------------------------

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
T Message::getValue(int pos) {
	// get a pointer to type T (use reinterpret_cast to force the type)
	if ((pos + sizeof(T)) <= this->size) {
		T value{};
		memcpy(&value, (this->message + pos), sizeof(T));
		return value;
	}
	return T{};
}

// write values to serial port
void Message::send() {
	for (int i = 0; i < this->size; i++) {
		Serial.write(*(this->message + i));
	}
}

// debug print function
void Message::debugPrint() {
	// construct debug strings and print statements
	// copy array of bytes to tmp array to avoid unintentional memory corruption
	uint8_t msgTmp[this->size];
	memcpy(msgTmp, this->message, this->size);

	Serial.print("Bytes in array (hex): ");
	String dbgStr = "";
	dbgStr += String(msgTmp[0], HEX);
	dbgStr += ",";
	dbgStr += String(msgTmp[1], HEX);
	dbgStr += "|";
	dbgStr += String(msgTmp[2], HEX);
	dbgStr += ",";
	dbgStr += String(msgTmp[3], HEX);
	dbgStr += ",";
	dbgStr += String(msgTmp[4], HEX);
	dbgStr += ",";
	dbgStr += String(msgTmp[5], HEX);
	dbgStr += ",";
	dbgStr += String(msgTmp[6], HEX);
	dbgStr += ",";
	dbgStr += String(msgTmp[7], HEX);
	dbgStr += ",";
	dbgStr += String(msgTmp[8], HEX);
	dbgStr += ",";
	dbgStr += String(msgTmp[9], HEX);
	dbgStr += "|";
	dbgStr += String(msgTmp[10], HEX);
	dbgStr += ",";
	dbgStr += String(msgTmp[11], HEX);
	dbgStr += "|";
	dbgStr += String(msgTmp[12], HEX);
	dbgStr += ",";
	dbgStr += String(msgTmp[13], HEX);
	dbgStr += "|";
	dbgStr += String(msgTmp[14], HEX);
	dbgStr += ",";
	dbgStr += String(msgTmp[15], HEX);
	dbgStr += "|";
	dbgStr += String(msgTmp[16], HEX);
	dbgStr += ",";
	dbgStr += String(msgTmp[17], HEX);
	dbgStr += "|";
	dbgStr += String(msgTmp[18], HEX);
	dbgStr += ",";
	dbgStr += String(msgTmp[19], HEX);
	dbgStr += "|";
	dbgStr += String(msgTmp[20], HEX);
	dbgStr += ",";
	dbgStr += String(msgTmp[21], HEX);
	Serial.print(dbgStr);

	Serial.println();
	Serial.print("Re-interpreted values (decimal): ");
	Serial.print(this->getValue<int16_t>(0));
	Serial.print("|");
	Serial.print(this->getValue<uint64_t>(2));
	Serial.print("|");
	Serial.print(this->getValue<int16_t>(10));
	Serial.print("|");
	Serial.print(this->getValue<int16_t>(12));
	Serial.print("|");
	Serial.print(this->getValue<int16_t>(14));
	Serial.print("|");
	Serial.print(this->getValue<int16_t>(16));
	Serial.print("|");
	Serial.print(this->getValue<int16_t>(18));
	Serial.print("|");
	Serial.print(this->getValue<int16_t>(20));
	Serial.println();
}
