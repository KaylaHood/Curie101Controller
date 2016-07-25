#pragma once

#define SERIAL_ENABLED 1

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

	Message();

	Message(int16_t s);

	Message& operator=(const Message& sm);

	~Message();

	template <typename T>
	void setValue(const T& val, int pos);

	template <typename T>
	T getValue(int pos) const;

	void send() const;

	void debugPrintMaster() const;

	void debugPrintSlave() const;
};

void setup();

void loop();

void updateValues(int16_t opcode);

void calibrate();

static void eventCallback(void);

bool isZeroMotion(uint64_t time, uint64_t zeroMotionTime);