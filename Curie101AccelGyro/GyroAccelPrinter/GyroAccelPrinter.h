#pragma once

// Serial / BLE toggle macro
// *** both cannot be enabled at once
#define SERIAL_ENABLED
#ifndef SERIAL_ENABLED
#define BLE_ENABLED
#endif

// define debug macros (comment out to turn off debug printing)
#ifdef SERIAL_ENABLED
//#define CURIE_SERIAL_DEBUG 
//#define CURIE_INTERRUPT_DEBUG
//#define CURIE_NOMO_DEBUG
//#define CURIE_CALIBRATION_DEBUG
//#define CURIE_SPEED_DEBUG
#endif

// struct to manage formatted message
struct Message {
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
	T getValue(int pos);

	// write values to serial port
	void send();

	// debug print function
	void debugPrint();
};

void setup();

void loop();

void updateValues(int16_t opcode);

void calibrate();

static void eventCallback(void);

bool isZeroMotion(uint64_t time, uint64_t zeroMotionTime);