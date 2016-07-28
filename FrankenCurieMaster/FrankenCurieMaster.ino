#include <CurieIMU.h>
#include <CurieBLE.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <limits.h>
#include <cstdint>
#include "FrankenCurieMaster.h"

#if SERIAL_ENABLED
enum Opcodes { Normal, ZeroMotion, Calibration, Slave1Calibration, Slave2Calibration };

bool serialIsConnected = false;

Message masterMsg = Message(46);
Message slave1Msg = Message(14);
Message slave2Msg = Message(14);

char calibrationMsg = 'a';
char slave1CalibrationMsg = 'b';
char slave2CalibrationMsg = 'c';
char disconnectMsg = 'd';
char connectMsg = 'y';
#endif

SoftwareSerial Slave1(3,2); // 3 = INPUT, 2 = OUTPUT
SoftwareSerial Slave2(5,4); // 5 = INPUT, 4 = OUTPUT

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

uint64_t microsPrevious;
uint64_t microsPerUpdate = (1000000 / (sampleRate / 4));
bool calibrateMasterAgain = false;
bool calibrateSlave1Again = false;
bool calibrateSlave2Again = false;

void setup() {
#if SERIAL_ENABLED
	Serial.begin(38400); // initialize Serial communication
	while (!Serial);    // wait for the serial port to open
#endif

	CurieIMU.begin();

	CurieIMU.attachInterrupt(eventCallback);
	CurieIMU.setDetectionThreshold(CURIE_IMU_ZERO_MOTION, zeroMotionThreshold);
	CurieIMU.setDetectionDuration(CURIE_IMU_ZERO_MOTION, zeroMotionDuration);
	CurieIMU.interrupts(CURIE_IMU_ZERO_MOTION);

	microsPrevious = micros();
	if (microsPerUpdate < 16667) {
		microsPerUpdate = 16667;
	}

	CurieIMU.setGyroRange(gyroRange);
	CurieIMU.setGyroRate(sampleRate);
	CurieIMU.setAccelerometerRange(accelRange);
	CurieIMU.setAccelerometerRate(sampleRate);
	CurieIMU.BMI160Class::setGyroDLPFMode(BMI160_DLPF_MODE_OSR4);
	CurieIMU.BMI160Class::setAccelDLPFMode(BMI160_DLPF_MODE_OSR4);

	pinMode(2, OUTPUT);
	pinMode(3, INPUT);
	pinMode(4, OUTPUT);
	pinMode(5, INPUT);
	Slave1.begin(38400);
	Slave2.begin(38400);

}

void loop() {
#if CURIE_SPEED_DEBUG
	Serial.print("starting loop time: ");
	Serial.print(micros());
	Serial.println();
#endif

#if SERIAL_ENABLED
	if (Serial.available() > 0) {
		char cmd = Serial.read();
		if (serialIsConnected) {
#if CURIE_CALIBRATION_DEBUG || CURIE_SERIAL_DEBUG
			Serial.print("Serial port was written to externally.");
			Serial.println();
#endif	
			if (cmd == calibrationMsg) {
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
			else if (cmd == slave1CalibrationMsg) {
#if CURIE_CALIBRATION_DEBUG
				Serial.print("Received \"");
				Serial.print(slave1CalibrationMsg);
				Serial.print("\".");
				Serial.println();
#endif
				calibrateSlave1();
			}
			else if (cmd == slave2CalibrationMsg) {
#if CURIE_CALIBRATION_DEBUG
				Serial.print("Received \"");
				Serial.print(slave2CalibrationMsg);
				Serial.print("\".");
				Serial.println();
#endif
				calibrateSlave2();
			}
			else if (cmd == disconnectMsg) {
#if CURIE_MASTER_DEBUG
				Serial.print("Received \"d\". Sending to slaves.");
				Serial.println();
#endif
				Slave1.write(disconnectMsg);
				Slave2.write(disconnectMsg);
				serialIsConnected = false;
			}
			else if (calibrateMasterAgain) {
#if CURIE_CALIBRATION_DEBUG
				Serial.print("calibrateMasterAgain");
				Serial.println();
#endif
				calibrateMasterAgain = false;
				zMT = zeroMotionDetectedMicros;
				microsNow = micros();
				calibrate();
			}
			else if (calibrateSlave1Again) {
#if CURIE_CALIBRATION_DEBUG
				Serial.print("calibrateSlave1Again");
				Serial.println();
#endif
				calibrateSlave1Again = false;
				calibrateSlave1();
			}
			else if (calibrateSlave2Again) {
#if CURIE_CALIBRATION_DEBUG
				Serial.print("calibrateSlave2Again");
				Serial.println();
#endif
				calibrateSlave2Again = false;
				calibrateSlave2();
			}
			else {
				updateValues(Normal);
			}
		}
		else if (cmd == connectMsg) {
#if CURIE_MASTER_DEBUG
			Serial.print("Received \"y\". Sending to slaves.");
			Serial.println();
#endif
			Slave1.write(connectMsg);
			Slave2.write(connectMsg);
			serialIsConnected = true;
		}
	}
	else if (serialIsConnected) {
		if (calibrateMasterAgain) {
#if CURIE_CALIBRATION_DEBUG
			Serial.print("calibrateMasterAgain");
			Serial.println();
#endif
			calibrateMasterAgain = false;
			zMT = zeroMotionDetectedMicros;
			microsNow = micros();
			calibrate();
		}
		else if (calibrateSlave1Again) {
#if CURIE_CALIBRATION_DEBUG
			Serial.print("calibrateSlave1Again");
			Serial.println();
#endif
			calibrateSlave1Again = false;
			calibrateSlave1();
		}
		else if (calibrateSlave2Again) {
#if CURIE_CALIBRATION_DEBUG
			Serial.print("calibrateSlave2Again");
			Serial.println();
#endif
			calibrateSlave2Again = false;
			calibrateSlave2();
		}
		else {
#if CURIE_MASTER_DEBUG
			Serial.print("updateValues normally");
			Serial.println();
#endif
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
#if CURIE_SPEED_DEBUG
	Serial.print("update start time: ");
	Serial.print(micros());
	Serial.println();
#endif
	int rawAx32, rawAy32, rawAz32;
	int rawGx32, rawGy32, rawGz32;
	int16_t rawAx16 = 0, rawAy16 = 0, rawAz16 = 0;
	int16_t rawGx16 = 0, rawGy16 = 0, rawGz16 = 0;

	while ((micros() - microsPrevious) < microsPerUpdate) {
#if CURIE_SPEED_DEBUG
		Serial.print("waiting...");
		Serial.println();
#endif
	}

	zMT = zeroMotionDetectedMicros;
	microsNow = micros();

	if (opcode != Slave1Calibration) {
		Slave1.listen();
		Slave1.write('u');
	}

	CurieIMU.readMotionSensor(
		rawAx32,
		rawAy32,
		rawAz32,
		rawGx32,
		rawGy32,
		rawGz32
	);
	
	if (opcode != Slave1Calibration) {
#if CURIE_MASTER_DEBUG
		Serial.print("Getting data from Slave 1.");
		Serial.println();
#endif
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
	}

	if (opcode != Slave2Calibration) {
		Slave2.listen();
		Slave2.write('u');
	}

	rawGx32 /= gyroSensitivity;
	rawGy32 /= gyroSensitivity;
	rawGz32 /= gyroSensitivity;

	rawAx16 = rawAx32;
	rawAy16 = rawAy32;
	rawAz16 = rawAz32;
	rawGx16 = rawGx32;
	rawGy16 = rawGy32;
	rawGz16 = rawGz32;

	if (opcode != Slave2Calibration) {
#if CURIE_MASTER_DEBUG
		Serial.print("Getting data from Slave 2.");
		Serial.println();
#endif
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
	}

	if ((isZeroMotion(microsNow, zMT) || slave1Msg.getValue<int16_t>(0) == ZeroMotion || slave2Msg.getValue<int16_t>(0) == ZeroMotion) && opcode < Calibration) {
		opcode = ZeroMotion;
	}

	masterMsg.setValue<int16_t>(opcode, 0);
	if (opcode == Calibration) {
		masterMsg.setValue<int16_t>(accelRange, 2);
		masterMsg.setValue<int16_t>(gyroRange, 4);
	}
	else {
		masterMsg.setValue<uint64_t>(microsNow, 2);
	}
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
	masterMsg.setValue<int16_t>(slave1Msg.getValue<int16_t>(8), 34);
	masterMsg.setValue<int16_t>(slave1Msg.getValue<int16_t>(10), 36);
	masterMsg.setValue<int16_t>(slave1Msg.getValue<int16_t>(12), 38);
	masterMsg.setValue<int16_t>(slave2Msg.getValue<int16_t>(8), 40);
	masterMsg.setValue<int16_t>(slave2Msg.getValue<int16_t>(10), 42);
	masterMsg.setValue<int16_t>(slave2Msg.getValue<int16_t>(12), 44);

#if CURIE_SERIAL_DEBUG == 0
	char peek = Serial.peek();
	if (opcode < Calibration && (peek == calibrationMsg || peek == slave1CalibrationMsg || peek == slave2CalibrationMsg)) {
		Serial.flush();
		return;
	}
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
	Serial.print("|");
	Serial.print(slave1Msg.getValue<int16_t>(8));
	Serial.print("|");
	Serial.print(slave1Msg.getValue<int16_t>(10));
	Serial.print("|");
	Serial.print(slave1Msg.getValue<int16_t>(12));
	Serial.print("|");
	Serial.print(slave2Msg.getValue<int16_t>(8));
	Serial.print("|");
	Serial.print(slave2Msg.getValue<int16_t>(10));
	Serial.print("|");
	Serial.print(slave2Msg.getValue<int16_t>(12));
	Serial.println();
	masterMsg.debugPrintMaster();
#endif

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
		CurieIMU.autoCalibrateGyroOffset();

		CurieIMU.autoCalibrateXAccelOffset(0);
		CurieIMU.autoCalibrateYAccelOffset(0);
		CurieIMU.autoCalibrateZAccelOffset(1);

		updateValues(Calibration);
	}
	else {
		calibrateMasterAgain = true;
	}
}

void calibrateSlave1() {
	Slave1.listen();
	Slave1.write((uint8_t)calibrationMsg);
	int tries = 0;
	while (Slave1.available() < slave1Msg.size); // wait for slave to send calibration message
	for (int i = 0; i < slave1Msg.size; i++) {
		slave1Msg.setValue<uint8_t>((uint8_t)Slave1.read(), i);
	}
	while ((slave1Msg.getValue<int16_t>(0) != Calibration) && tries < 10) {
		while (Slave1.available() < slave1Msg.size);
		for (int i = 0; i < slave1Msg.size; i++) {
			slave1Msg.setValue<uint8_t>((uint8_t)Slave1.read(), i);
		}
		tries += 1;
	}
	if (tries >= 10) {
		calibrateSlave1Again = true;
	}
	else {
		updateValues(Slave1Calibration);
	}
}

void calibrateSlave2() {
	Slave2.listen();
	Slave2.write((uint8_t)calibrationMsg);
	int tries = 0;
	while (Slave2.available() < slave2Msg.size); // wait for slave to send calibration message
	for (int i = 0; i < slave2Msg.size; i++) {
		slave2Msg.setValue<uint8_t>((uint8_t)Slave2.read(), i);
	}
	while ((slave2Msg.getValue<int16_t>(0) != Calibration) && tries < 20) {
		while (Slave2.available() < slave2Msg.size);
		for (int i = 0; i < slave2Msg.size; i++) {
			slave2Msg.setValue<uint8_t>((uint8_t)Slave2.read(), i);
		}
		tries += 1;
	}
	if (tries >= 10) {
		calibrateSlave2Again = true;
	}
	else {
		updateValues(Slave2Calibration);
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
		Serial.write(*(this->message + i));
	}
}

void Message::debugPrintMaster() const {
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

void Message::debugPrintSlave() const {
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