#include <CurieIMU.h>
#include <CurieBLE.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <limits.h>
#include <cstdint>
#include "FrankenCurieMaster.h"

#if SERIAL_ENABLED
// -----------------------------------
// Opcode Enum for Serial Port Opcodes
// -----------------------------------
enum Opcodes { Normal, ZeroMotion, Calibration };

// boolean for indicating computer's presence on SerialPort
bool serialIsConnected = false;

// message holder for communication with computer using a byte array
Message masterMsg = Message(34);
// slave message holders
Message slave1Msg = Message(8);
Message slave2Msg = Message(8);

char calibrationMsg = 'c';
#endif

#if BLE_ENABLED
// -----------------------------------
// BLE variables
// -----------------------------------
BLEPeripheral blePeripheral;
BLEService IMUService("19B10010-E8F2-537E-4F6C-D104768A1214");
int characteristicSize = 20;
BLECharacteristic CalibMaster("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite, 20);
BLECharacteristic CalibSlaves("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite, 20);
BLECharacteristic MotionMaster("19B10013-E8F2-537E-4F6C-D104768A1214", BLENotify | BLERead | BLEWrite, 20);
BLECharacteristic MotionSlaves("19B10014-E8F2-537E-4F6C-D104768A1214", BLENotify | BLERead | BLEWrite, 20);
BLECharacteristic ZeroMotMaster("19B10015-E8F2-537E-4F6C-D104768A1214", BLENotify | BLERead | BLEWrite, 20);
BLECharacteristic ZeroMotSlaves("19B10016-E8F2-537E-4F6C-D104768A1214", BLENotify | BLERead | BLEWrite, 20);
// boolean for indicating computer's presence on BLE
bool bleIsConnected = false;

// message holder for master characteristic
Message masterMsg = Message(20);
// message holder for slave characteristic
Message slaveMsg = Message(20);
#endif

// -----------------------------------------------------------
// Software Serial Objects for Communication over Digitial I/O
// -----------------------------------------------------------
SoftwareSerial Slave1(3,2); // 3 = INPUT, 2 = OUTPUT
SoftwareSerial Slave2(5,4); // 5 = INPUT, 4 = OUTPUT

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
// *** also cannot be less than 16,667 (no faster than 60 Hz)
//     This value is raised to 16,667 in setup() if it is less than that.
uint64_t microsPerUpdate = (1000000 / (sampleRate / 4));
// if calibration fails, set this to true so that next loop knows to attempt calibration again
bool calibrateAgain = false;

void setup() {
#if SERIAL_ENABLED
	Serial.begin(38400); // initialize Serial communication
	while (!Serial);    // wait for the serial port to open
#endif

	// initialize device
	CurieIMU.begin();

	// Set up zero motion interrupts
	CurieIMU.attachInterrupt(eventCallback);
	CurieIMU.setDetectionThreshold(CURIE_IMU_ZERO_MOTION, zeroMotionThreshold);
	CurieIMU.setDetectionDuration(CURIE_IMU_ZERO_MOTION, zeroMotionDuration);
	CurieIMU.interrupts(CURIE_IMU_ZERO_MOTION);

	// initialize time
	microsPrevious = micros();
	// set microsPerUpdate to minimum value if it is less than that (16,667 microseconds)
	// *** this makes sure updates happen no faster than 60 hz
	if (microsPerUpdate < 16667) {
		microsPerUpdate = 16667;
	}

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

	// Set up the SoftwareSerial objects for slave communication
	pinMode(2, OUTPUT);
	pinMode(3, INPUT);
	pinMode(4, OUTPUT);
	pinMode(5, INPUT);
	Slave1.begin(38400);
	Slave2.begin(38400);

#if BLE_ENABLED
	// -----------------------------------
	// BLE set-up
	// -----------------------------------

	// Set the local name that the peripheral advertises
	blePeripheral.setLocalName("CurieIMU");
	blePeripheral.setAdvertisedServiceUuid(IMUService.uuid());

	// add service and characteristics
	blePeripheral.addAttribute(IMUService);
	blePeripheral.addAttribute(CalibMaster);
	blePeripheral.addAttribute(CalibSlaves);
	blePeripheral.addAttribute(MotionMaster);
	blePeripheral.addAttribute(MotionSlaves);
	blePeripheral.addAttribute(ZeroMotMaster);
	blePeripheral.addAttribute(ZeroMotSlaves);

	// initialize characteristics to zero
	CalibMaster.setValue(0);
	CalibSlave.setValue(0);
	MotionMaster.setValue(0);
	MotionSlave.setValue(0);
	ZeroMotMaster.setValue(0);
	ZeroMotSlave.setValue(0);

	// advertise the service
	blePeripheral.begin();
#endif
}

void loop() {
#if CURIE_SPEED_DEBUG
	Serial.print("starting loop time: ");
	Serial.print(micros());
	Serial.println();
#endif

#if BLE_ENABLED
	// listen for BLE peripherals to connect
	BLECentral bleCentral = blePeripheral.central();
	// set toggle for BLE connection
	if (bleCentral) {
		bleIsConnected = true;
		if (CalibMaster.written()) {
#if CURIE_CALIBRATION_DEBUG || CURIE_BLE_DEBUG
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

#if SERIAL_ENABLED
	if (Serial.available() > 0) {
		// read request from the serial port
		char cmd = Serial.read();
		if (serialIsConnected) {
#if CURIE_CALIBRATION_DEBUG || CURIE_SERIAL_DEBUG
			Serial.print("Serial port was written to externally.");
			Serial.println();
#endif	
			if (cmd == calibrationMsg) {
				// Computer is requesting calibration
				// save last zero motion time before setting "microsNow"
#if CURIE_CALIBRATION_DEBUG
				Serial.print("Received \"");
				Serial.print(calibrationMsg);
				Serial.print("\".");
				Serial.println();
#endif
				zMT = zeroMotionDetectedMicros;
				microsNow = micros();
				calibrate();
			}
			else if (cmd == 'd') {
				// Computer has disconnected
				// send same command to slaves
#if CURIE_MASTER_DEBUG
				Serial.print("Received \"d\". Sending to slaves.");
				Serial.println();
#endif
				Slave1.write('d');
				Slave2.write('d');
				// set connection boolean to false
				serialIsConnected = false;
			}
			else if (calibrateAgain) {
#if CURIE_CALIBRATION_DEBUG
				Serial.print("calibrateAgain");
				Serial.println();
#endif
				calibrateAgain = false;
				// save last zero motion time before setting "microsNow"
				zMT = zeroMotionDetectedMicros;
				microsNow = micros();
				calibrate();
			}
			else {
				updateValues(Normal);
			}
		}
		else if (cmd == 'y') {
			// Computer has requested communication to begin
			// Send same command to slaves
#if CURIE_MASTER_DEBUG
			Serial.print("Received \"y\". Sending to slaves.");
			Serial.println();
#endif
			Slave1.write('y');
			Slave2.write('y');
			// set connection boolean to true
			serialIsConnected = true;
		}
	}
	else if (serialIsConnected) {
		if (calibrateAgain) {
#if CURIE_CALIBRATION_DEBUG
			Serial.print("calibrateAgain");
			Serial.println();
#endif
			calibrateAgain = false;
			// save last zero motion time before setting "microsNow"
			zMT = zeroMotionDetectedMicros;
			microsNow = micros();
			calibrate();
		}
		else {
#if CURIE_MASTER_DEBUG
			Serial.print("updateValues normally");
			Serial.println();
#endif
			// no commands are being received, update normally
			updateValues(Normal);
		}
	}
#endif

#if CURIE_SPEED_DEBUG
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
#if CURIE_SPEED_DEBUG
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
#if CURIE_SPEED_DEBUG
		Serial.print("waiting...");
		Serial.println();
#endif
	}

	// save most recent zero motion detected time, because the zero motion interrupt *could*
	// fire inbetween setting "microsNow" and testing for zero motion, which would result
	// in the function "isZeroMotion" returning false when zero motion is happening.
	zMT = zeroMotionDetectedMicros;
	microsNow = micros();

	// ask first slave for data, then do work while slave sends data
	Slave1.listen();
	Slave1.write('u');

	// read raw measurements from device 
	// *** (IMU library returns 32-bit ints for IMU data, even though hardware data is 16-bit ints)
	CurieIMU.readMotionSensor(
		rawAx32,
		rawAy32,
		rawAz32,
		rawGx32,
		rawGy32,
		rawGz32
	);
	
#if CURIE_MASTER_DEBUG
	Serial.print("Getting data from Slave 1.");
	Serial.println();
#endif
	// gather first slave's data (wait for transmission to complete first)
	while (Slave1.available() < slave1Msg.size) {
#if CURIE_MASTER_DEBUG
		Serial.print("Waiting for full message. Available now: ");
		Serial.print(Slave1.available());
		Serial.println();
#endif
	}
	for (int i = 0; i < slave1Msg.size; i++) {
		slave1Msg.setValue<uint8_t>((uint8_t)Slave1.read(), i);
	}

	// request data from second slave
	Slave2.listen();
	Slave2.write('u');

	// divide gyro data by sensitivity constant
	rawGx32 /= gyroSensitivity;
	rawGy32 /= gyroSensitivity;
	rawGz32 /= gyroSensitivity;

	// do conversion to 16 bits 
	// *** (Curie's hardware IMU device returns 16-bit values for data, no information is lost here)
	rawAx16 = rawAx32;
	rawAy16 = rawAy32;
	rawAz16 = rawAz32;
	rawGx16 = rawGx32;
	rawGy16 = rawGy32;
	rawGz16 = rawGz32;

#if CURIE_MASTER_DEBUG
	Serial.print("Getting data from Slave 2.");
	Serial.println();
#endif
	// get data from second slave
	while (Slave2.available() < slave2Msg.size) {
#if CURIE_MASTER_DEBUG
		Serial.print("Waiting for full message. Available now: ");
		Serial.print(Slave2.available());
		Serial.println();
#endif
	}
	for (int i = 0; i < slave2Msg.size; i++) {
		slave2Msg.setValue<uint8_t>((uint8_t)Slave2.read(), i);
	}

	// if zero motion event is occurring and the opcode is not 0
	// (meaning this is not a calibration update), then set opcode
	// to 2, which is the zero-motion opcode.
	if ((isZeroMotion(microsNow, zMT) || slave1Msg.getValue<int16_t>(0) == ZeroMotion || slave2Msg.getValue<int16_t>(0) == ZeroMotion) && opcode != Calibration) {
		opcode = ZeroMotion;
	}

	// else, if this is calibration, set rawGx to accelerometer range, and rawGy to gyroscope range
	else if (opcode == Calibration) {
		rawGx16 = accelRange;
		rawGy16 = gyroRange;
	}

	// set up values in statement
	masterMsg.setValue<int16_t>(opcode, 0);
	masterMsg.setValue<uint64_t>(microsNow, 2);
	masterMsg.setValue<int16_t>(rawAx16, 10);
	masterMsg.setValue<int16_t>(rawAy16, 12);
	masterMsg.setValue<int16_t>(rawAz16, 14);
	masterMsg.setValue<int16_t>(slave1Msg.getValue<int16_t>(2), 16);
	masterMsg.setValue<int16_t>(slave1Msg.getValue<int16_t>(4), 18);
	masterMsg.setValue<int16_t>(slave1Msg.getValue<int16_t>(6), 20);
	masterMsg.setValue<int16_t>(slave2Msg.getValue<int16_t>(2), 22);
	masterMsg.setValue<int16_t>(slave2Msg.getValue<int16_t>(4), 24);
	masterMsg.setValue<int16_t>(slave2Msg.getValue<int16_t>(6), 26);
	masterMsg.setValue<int16_t>(rawGx16, 28);
	masterMsg.setValue<int16_t>(rawGy16, 30);
	masterMsg.setValue<int16_t>(rawGz16, 32);

	// -----------------------------------
	// BLE Characteristic and Serial Port Update
	// -----------------------------------
	// syntax of 20-byte "master" characteristic (BLE only):
	// -----
	// [8 bytes for timestamp][six-byte Accel data from master][six-byte Gyro data from master]
	// -----                         (three 16-bit ints)              (three 16-bit ints)
	// -----
	// syntax of 20-byte "slave" characteristic (BLE only):
	// -----
	// [8 bytes for timestamp][six-byte Accel data from slave 1][six-byte Accel data from slave 2]
	// -----                         (three 16-bit ints)              (three 16-bit ints)
	// -----

#if BLE_ENABLED
	// check for connected central
	if (bleIsConnected) {
		// construct unsigned char array of values because the BLE "setValue" function 
		// requires an unsigned char array
		unsigned char ucvalues[characteristicSize];
		// use memcpy because we don't want the byte data to be interpreted into chars. 
		// We just want the binary representation of those bytes.
		// *** skip the opcode in the statement (our memcpy's source)
		// *** (we can only fit 20 bytes into the BLE characteristic)
		memcpy(ucvalues, (statement.message + 2), characteristicSize);

		// update characteristic value
		if (opcode == Calibration) {
			/* TODO
			CalibMaster.setValue();
			CalibSlaves.setValue();
			*/
		}
		else if (opcode == Normal) {
			/* TODO
			MotionMaster.setValue();
			MotionSlaves.setValue();
			*/
		}
		else if (opcode == ZeroMotion) {
			/* TODO
			ZeroMotMaster.setValue();
			ZeroMotSlaves.setValue();
			*/
		}
	}
#endif

#if CURIE_SERIAL_DEBUG == 0
	// before sending data, check for calibration request
	// *** only stop updating if not already calibrating
	if (Serial.peek() == calibrationMsg && opcode != Calibration) {
		// calibration request received! Abort update
		Serial.flush();
		return;
	}
	// write byte values to Serial Port
	masterMsg.send();
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
	Serial.print(slave1Msg.getValue<int16_t>(2));
	Serial.print("|");
	Serial.print(slave1Msg.getValue<int16_t>(4));
	Serial.print("|");
	Serial.print(slave1Msg.getValue<int16_t>(6));
	Serial.print("|");
	Serial.print(slave2Msg.getValue<int16_t>(2));
	Serial.print("|");
	Serial.print(slave2Msg.getValue<int16_t>(4));
	Serial.print("|");
	Serial.print(slave2Msg.getValue<int16_t>(6));
	Serial.print("|");
	Serial.print(rawGx16);
	Serial.print("|");
	Serial.print(rawGy16);
	Serial.print("|");
	Serial.print(rawGz16);
	Serial.println();
	masterMsg.debugPrintMaster();
#endif

	// update the time keeping variable
	microsPrevious = microsNow;
#if CURIE_SPEED_DEBUG
	Serial.print("update end time: ");
	Serial.print(micros());
	Serial.println();
#endif
#if SERIAL_ENABLED
	Serial.flush();
#endif
}

void calibrate() {
	if (isZeroMotion(microsNow, zMT)) {
		// calibrate gyroscope
		CurieIMU.autoCalibrateGyroOffset();

		// calibrate accelerometer
		CurieIMU.autoCalibrateXAccelOffset(0);
		CurieIMU.autoCalibrateYAccelOffset(0);
		CurieIMU.autoCalibrateZAccelOffset(1);

		// calibrate slaves individually (to prevent simultaneous digital i/o writes)
		Slave1.listen();
		Slave1.write((uint8_t)calibrationMsg);
		while (Slave1.available() < slave1Msg.size); // wait for slave to send calibration message
		int tries = 0;
		for (int i = 0; i < slave1Msg.size; i++) {
			slave1Msg.setValue<uint8_t>((uint8_t)Slave1.read(), i);
		}
		while ((slave1Msg.getValue<int16_t>(0) != Calibration) && tries < 10) {
			Slave1.flush();
			Slave1.print(calibrationMsg);
			while (Slave1.available() < slave1Msg.size);
			for (int i = 0; i < slave1Msg.size; i++) {
				slave1Msg.setValue<uint8_t>((uint8_t)Slave1.read(), i);
			}
			tries += 1;
		}

		Slave2.listen();
		Slave2.write((uint8_t)calibrationMsg);
		while (Slave2.available() < slave2Msg.size); // wait for slave to send calibration message
		tries = 0;
		for (int i = 0; i < slave2Msg.size; i++) {
			slave2Msg.setValue<uint8_t>((uint8_t)Slave2.read(), i);
		}
		
		while ((slave2Msg.getValue<int16_t>(0) != Calibration) && tries < 10) {
			Slave2.flush();
			Slave2.print(calibrationMsg);
			while (Slave2.available() < slave2Msg.size);
			for (int i = 0; i < slave2Msg.size; i++) {
				slave2Msg.setValue<uint8_t>((uint8_t)Slave2.read(), i);
			}
			tries += 1;
		}
		
		if (tries >= 10) {
			// one or more slaves failed to calibrate
			calibrateAgain = true;
		}
		else {
			// finish calibrating by updating sensor values
			updateValues(Calibration);
		}
	}
	else {
		// zero motion not detected, try again on next loop
		calibrateAgain = true;
	}
}

static void eventCallback(void) {
	if (CurieIMU.getInterruptStatus(CURIE_IMU_ZERO_MOTION)) {
		zeroMotionDetectedMicros = micros();
#if CURIE_INTERRUPT_DEBUG
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
#if CURIE_NOMO_DEBUG
	Serial.print("Update time: ");
	Serial.print(time);
	Serial.print(", zMT: ");
	Serial.print(zeroMotionTime);
	Serial.println();
#endif	
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
		Serial.write(*(this->message + i));
	}
}

// debug print function for master message
void Message::debugPrintMaster() const {
	// construct debug strings and print to serial port
	// copy array of bytes to tmp array to avoid unintentional memory corruption
	uint8_t mp[this->size];
	memcpy(mp, this->message, this->size);

	Serial.print("Bytes in array (hex): ");
	String dbgStr = "";
	dbgStr += String(mp[0], HEX);
	dbgStr += ",";
	dbgStr += String(mp[1], HEX);
	dbgStr += "|";
	dbgStr += String(mp[2], HEX);
	dbgStr += ",";
	dbgStr += String(mp[3], HEX);
	dbgStr += ",";
	dbgStr += String(mp[4], HEX);
	dbgStr += ",";
	dbgStr += String(mp[5], HEX);
	dbgStr += ",";
	dbgStr += String(mp[6], HEX);
	dbgStr += ",";
	dbgStr += String(mp[7], HEX);
	dbgStr += ",";
	dbgStr += String(mp[8], HEX);
	dbgStr += ",";
	dbgStr += String(mp[9], HEX);
	dbgStr += "|";
	dbgStr += String(mp[10], HEX);
	dbgStr += ",";
	dbgStr += String(mp[11], HEX);
	dbgStr += "|";
	dbgStr += String(mp[12], HEX);
	dbgStr += ",";
	dbgStr += String(mp[13], HEX);
	dbgStr += "|";
	dbgStr += String(mp[14], HEX);
	dbgStr += ",";
	dbgStr += String(mp[15], HEX);
	dbgStr += "|";
	dbgStr += String(mp[16], HEX);
	dbgStr += ",";
	dbgStr += String(mp[17], HEX);
	dbgStr += "|";
	dbgStr += String(mp[18], HEX);
	dbgStr += ",";
	dbgStr += String(mp[19], HEX);
	dbgStr += "|";
	dbgStr += String(mp[20], HEX);
	dbgStr += ",";
	dbgStr += String(mp[21], HEX);
	dbgStr += "|";
	dbgStr += String(mp[22], HEX);
	dbgStr += ",";
	dbgStr += String(mp[23], HEX);
	dbgStr += "|";
	dbgStr += String(mp[24], HEX);
	dbgStr += ",";
	dbgStr += String(mp[25], HEX);
	dbgStr += "|";
	dbgStr += String(mp[26], HEX);
	dbgStr += ",";
	dbgStr += String(mp[27], HEX);
	dbgStr += "|";
	dbgStr += String(mp[28], HEX);
	dbgStr += ",";
	dbgStr += String(mp[29], HEX);
	dbgStr += "|";
	dbgStr += String(mp[30], HEX);
	dbgStr += ",";
	dbgStr += String(mp[31], HEX);
	dbgStr += "|";
	dbgStr += String(mp[32], HEX);
	dbgStr += ",";
	dbgStr += String(mp[33], HEX);
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
	Serial.print("|");
	Serial.print(this->getValue<int16_t>(22));
	Serial.print("|");
	Serial.print(this->getValue<int16_t>(24));
	Serial.print("|");
	Serial.print(this->getValue<int16_t>(26));
	Serial.print("|");
	Serial.print(this->getValue<int16_t>(28));
	Serial.print("|");
	Serial.print(this->getValue<int16_t>(30));
	Serial.print("|");
	Serial.print(this->getValue<int16_t>(32));
	Serial.println();
}

// debug print function for slave message
void Message::debugPrintSlave() const {
	// construct debug strings and print to serial port
	// copy array of bytes to tmp array to avoid unintentional memory corruption
	uint8_t mp[this->size];
	memcpy(mp, this->message, this->size);

	Serial.print("Bytes in array (hex): ");
	String dbgStr = "";
	dbgStr += String(mp[0], HEX);
	dbgStr += ",";
	dbgStr += String(mp[1], HEX);
	dbgStr += "|";
	dbgStr += String(mp[2], HEX);
	dbgStr += ",";
	dbgStr += String(mp[3], HEX);
	dbgStr += "|";
	dbgStr += String(mp[4], HEX);
	dbgStr += ",";
	dbgStr += String(mp[5], HEX);
	dbgStr += "|";
	dbgStr += String(mp[6], HEX);
	dbgStr += ",";
	dbgStr += String(mp[7], HEX);
	Serial.print(dbgStr);

	Serial.println();
	Serial.print("Re-interpreted values (decimal): ");
	Serial.print(this->getValue<int16_t>(0));
	Serial.print("|");
	Serial.print(this->getValue<int16_t>(2));
	Serial.print("|");
	Serial.print(this->getValue<int16_t>(4));
	Serial.print("|");
	Serial.print(this->getValue<int16_t>(6));
	Serial.println();
}

// debug print function for BLE message
// *** be sure to have the serial port open and set up before running this
void Message::debugPrintBLE() const {
	// construct debug strings and print to serial port
	// copy array of bytes to tmp array to avoid unintentional memory corruption
	uint8_t mp[this->size];
	memcpy(mp, this->message, this->size);

	Serial.print("Bytes in array (hex): ");
	String dbgStr = "";
	dbgStr += String(mp[0], HEX);
	dbgStr += ",";
	dbgStr += String(mp[1], HEX);
	dbgStr += ",";
	dbgStr += String(mp[2], HEX);
	dbgStr += ",";
	dbgStr += String(mp[3], HEX);
	dbgStr += ",";
	dbgStr += String(mp[4], HEX);
	dbgStr += ",";
	dbgStr += String(mp[5], HEX);
	dbgStr += ",";
	dbgStr += String(mp[6], HEX);
	dbgStr += ",";
	dbgStr += String(mp[7], HEX);
	dbgStr += "|";
	dbgStr += String(mp[8], HEX);
	dbgStr += ",";
	dbgStr += String(mp[9], HEX);
	dbgStr += "|";
	dbgStr += String(mp[10], HEX);
	dbgStr += ",";
	dbgStr += String(mp[11], HEX);
	dbgStr += "|";
	dbgStr += String(mp[12], HEX);
	dbgStr += ",";
	dbgStr += String(mp[13], HEX);
	dbgStr += "|";
	dbgStr += String(mp[14], HEX);
	dbgStr += ",";
	dbgStr += String(mp[15], HEX);
	dbgStr += "|";
	dbgStr += String(mp[16], HEX);
	dbgStr += ",";
	dbgStr += String(mp[17], HEX);
	dbgStr += "|";
	dbgStr += String(mp[18], HEX);
	dbgStr += ",";
	dbgStr += String(mp[19], HEX);
	Serial.print(dbgStr);

	Serial.println();
	Serial.print("Re-interpreted values (decimal): ");
	Serial.print(this->getValue<uint64_t>(0));
	Serial.print("|");
	Serial.print(this->getValue<int16_t>(8));
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
	Serial.println();
}
