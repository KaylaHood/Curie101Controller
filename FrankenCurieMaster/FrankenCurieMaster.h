#pragma once

// Serial / BLE toggle macro (0 = off)
// *** both cannot be enabled at once
#define SERIAL_ENABLED 1
#define BLE_ENABLED 0

// define debug macros (0 = off)
#define CURIE_SERIAL_DEBUG 0
#define CURIE_INTERRUPT_DEBUG 0
#define CURIE_NOMO_DEBUG 0
#define CURIE_CALIBRATION_DEBUG 0
#define CURIE_SPEED_DEBUG 0
#define CURIE_MASTER_DEBUG 0

class Message {
public:
	uint8_t* message; // pointer to array of bytes
	int16_t size; // size of byte array

	// default constructor
	Message();

	// constructor with size argument
	Message(int16_t s);

	Message& operator=(const Message& sm);

	// destructor
	~Message();

	// byte setting functions
	template <typename T>
	void setValue(const T& val, int pos);

	// retrieval functions
	template <typename T>
	T getValue(int pos) const;

	// write values to serial port
	void send() const;

	// debug print function for master message
	void debugPrintMaster() const;

	// debug print function for BLE message
	void debugPrintBLE() const;

	// debug print function for slave message
	void debugPrintSlave() const;
};

void setup();

void loop();

void updateValues(int16_t opcode);

void calibrate();

static void eventCallback(void);

bool isZeroMotion(uint64_t time, uint64_t zeroMotionTime);