#include <CurieIMU.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <math.h>
#include "CPUTMath.h" // Source: https://github.com/GameTechDev/ChatHeads/blob/master/CPUT/include/CPUTMath.h
#include "GyroAccelPrinter.h"

// Kalman filter for X axis of gyroscope
Kalman kalmanX;
// Kalman filter for Y axis of gyroscope
Kalman kalmanY;
// Kalman filter for Z axis of gyroscope
Kalman kalmanZ;

// program time in microseconds of last "update" call
unsigned long microsNow;
// program time in microseconds of last zero motion interrupt
unsigned long zeroMotionDetectedMicros = 100000000; // large to prevent unintentional trigger
// interval in microseconds for zero motion detection in update function
// if abs(zeroMotionDetectedMicros - microsNow) < interval, then zero motion is reported
unsigned int interval = 1280000;

// the threshold in micro Gs of the zero motion detection hardware
float zeroMotionThreshold = 1500.0f;
// the duration in seconds of zero motion detection 
// needed for the Curie to fire an interrupt 
float zeroMotionDuration = 5.0f;

// the range in degrees/second of the gyroscope
int gyroRange = 500;
// the range in Gs of the accelerometer
int accelRange = 2;
// the sample rate in Hz of multiple units
// including the filter, the gyroscope, and the accelerometer
int sampleRate = 25;
// the sensitivity of the gyroscope
// the raw values from the gyro are divided
// by this value before being passed into
// the filter in order to reduce the impact
// of small changes in motion.
// **NEEDS TO BE TWEAKED
int gyroSensitivity = 80;

// filtered x y z vector for rotation (in degrees)
float3 rotation;
// filtered x y z vector for acceleration (in Gs)
float3 accel;

// program time in microseconds of previous update
unsigned long microsPrevious;
// "tick" rate of update
// "updateValues" function restricts occurrence
// of updates to match the interval defined by this variable
unsigned long microsPerReading = 1000000 / sampleRate;

// gravity force on the local axes found during no motion
float3 gravity;
// vector of acceleration after removing effect of gravity (translational acceleration only)
float3 accT;

/*
Most recently computed orientation state
	0: flat, processor facing up
	1: flat, processor facing down
	2: landscape, analog pins down
	3: landscape, analog pins up
	4: portrait, USB connector up
	5: portrait, USB connector down
*/
int orientation;

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

	// initialize accel and rotation vectors as float3 instances
	accel = float3(0.0f);
	rotation = float3(0.0f);

	// note - do not calibrate yet
	// the Curie board might not be flat and might not be stationary
	// and the user may not want to calibrate immediately
}

void loop() {
	if (Serial.available() > 0) {
		char command = char(Serial.read());
		if (command == 'u') {
			// update values from curie
			updateValues(true,false);
		}
		else if (command == 'c') {
			calibrate();
		}
	}
	else return;

	//delay(50);
}

void updateValues(bool printingEnabled, bool detectZeroMotionEnabled) {
	// **if "printingEnabled" is true, then this
	// function will print the results
	// **if "detectZeroMotion" is true, this function will
	// use the zero motion interrupts to detect zero
	// motion from the Curie.
	int rawGx, rawGy, rawGz, rawAx, rawAy, rawAz;
	
	microsNow = micros();
	// block until correct time
	while (microsNow - microsPrevious < microsPerReading) { }

	// if zero motion is detected within [interval] microseconds
	if (isZeroMotion() && detectZeroMotionEnabled) {
		// keep in mind, since the microseconds are sampled
		// for a second time inside of the "isZeroMotion"
		// function, if a zero motion interrupt is fired
		// exactly [interval - n] microseconds before 
		// the first sampling, where "n" is the amount of
		// microseconds between the first sampling and the
		// second sampling within "isZeroMotion", then the
		// zero motion interrupt will not be caught.
		if (printingEnabled) {
			Serial.print("Zero Motion");
			Serial.println();
		}
		microsPrevious = microsPrevious + microsPerReading;
		return;
	}

	else {
		// read raw measurements from device
		CurieIMU.readMotionSensor
		(
			rawAx, 
			rawAy, 
			rawAz, 
			rawGx, 
			rawGy, 
			rawGz
		);

		// initialize converted gyro value vector
		float3 convGyro = convertRawGyro(rawGx/gyroSensitivity, rawGy/gyroSensitivity, rawGz/gyroSensitivity);

		// calculate roll (x-axis rotation) using atan2 with raw accelerometer 
		// values, then convert from radians to degrees
		double roll = radToDeg(atan2(double(rawAy), double(rawAz)));
		// calculate pitch (y-axis rotation) using atan with raw accelerometer
		// values, then convert from radians to degrees
		double pitch = radToDeg(atan((double(-(rawAx))) / sqrt(double(rawAy * rawAy + rawAz * rawAz))));
		// calculate yaw (z-axis rotation) using atan with raw accelerometer
		// values, then convert from radians to degrees
		double yaw = radToDeg(atan((double(-(rawAz))) / sqrt(double(rawAy * rawAy + rawAx * rawAx))));

		
		Serial.print("pitch, roll, and yaw:");
		Serial.println();
		printFloat3(float3(pitch, roll, yaw));
		

		// this fixes the transistion problem when the accelerometer angle jumps between -180 and 180 degrees
		// first set the y-axis rotation, or the roll
		if ((roll < -90 && rotation.x > 90) || (roll > 90 && rotation.x < -90)) 
		{
			kalmanX.setAngle(roll);
			rotation.x = roll;
		}
		else 
		{
			rotation.x = kalmanX.getAngle(roll, convGyro.x, (microsNow - microsPrevious));
		}
		// now set x-axis rotation, or pitch
		if (abs(rotation.x) > 90) {
			// invert rate, so it fits restricted accelerometer reading
			convGyro.y = -convGyro.y;
		}
		rotation.y = kalmanY.getAngle(pitch, convGyro.y, (microsNow - microsPrevious));
		// finally, set z-axis rotation, or the yaw
		rotation.z = kalmanZ.getAngle(yaw, convGyro.z, (microsNow - microsPrevious));

		// set accel vector to the G forces reported by Curie
		// conversion is needed to get proper units
		accel = convertRawAcceleration(rawAx, rawAy, rawAz);
		// calculate the local force of gravity on the x y z axes
		float3 rotGrav = rotateGravity(gravity, rotation);
		
		
		Serial.print("rotated gravity:");
		Serial.println();
		printFloat3(rotGrav);
		

		// save the adjusted-for-gravity values in
		// a separate float3 variable. (accT = "acceleration translational")
		accT = accel - rotGrav;

		if (printingEnabled) {
			// print comma-separated accelerometer x/y/z values
			
			Serial.print("acceleration:");
			Serial.println();
			
			printFloat3(accT);
			// print comma-separated pitch, roll, yaw values
			
			Serial.print("rotation in angles:");
			Serial.println();
			
			printFloat3(rotation);
		}
	}
	// update the time keeping variable
	microsPrevious = microsPrevious + microsPerReading;
}

void calibrate() {
	if (isZeroMotion()) {
		// calibrate gyroscope
		CurieIMU.autoCalibrateGyroOffset();
		
		// initizalize rotation vector to origin
		rotation = float3(0.0f);
		
		// calibrate accelerometer
		CurieIMU.autoCalibrateXAccelOffset(0);
		CurieIMU.autoCalibrateYAccelOffset(0);
		CurieIMU.autoCalibrateZAccelOffset(1);
		
		// update values of accelerometer and gyroscope vectors
		// pass (false,false) to disable printing and zero-motion detection
		updateValues(false,false);
		
		// detect and store force of gravity on Curie IMU
		// (stored in "gravity" vector and in "gravityTotal" float)
		setGravity();

		// store raw accelerometer values in vector for easy access
		float3 rawAccel = float3(CurieIMU.getAccelerationX(), CurieIMU.getAccelerationY(), CurieIMU.getAccelerationZ());
		// calculate roll (x-axis rotation) using atan2 with raw accelerometer 
		// values, then convert from radians to degrees
		double roll = radToDeg(atan2(rawAccel.y, rawAccel.z));
		// calculate pitch (y-axis rotation) using atan with raw accelerometer
		// values, then convert from radians to degrees
		double pitch = radToDeg(atan((-(rawAccel.x)) / sqrt(rawAccel.y * rawAccel.y + rawAccel.z * rawAccel.z)));
		// calculate yaw (z-axis rotation) using atan with raw accelerometer
		// values, then convert from radians to degrees
		double yaw = radToDeg(atan((double((accel.z))) / sqrt(double(accel.y * accel.y + accel.x * accel.x))));

		// set up Kalman filter
		kalmanX.setAngle(roll);
		kalmanY.setAngle(pitch);
		kalmanZ.setAngle(yaw);
		
		// report that calibration is finished to the Serial port
		Serial.print("calibrated");
		Serial.println();
	}
	else {
		// report that calibration was not completed to the Serial port
		Serial.print("cannot calibrate, motion detected");
		Serial.println();
	}
}

void setOrientation() {
	// ** board must be stationary for this to work **
	String orientationString = "";
	/*
	  0: flat, processor facing up
	  1: flat, processor facing down
	  2: landscape, analog pins down
	  3: landscape, analog pins up
	  4: portrait, USB connector up
	  5: portrait, USB connector down
	*/
	// calculate the absolute values of acceleration,
	// to determine the largest.
	float3 absAcc = float3
	(
		abs(accel.x),
		abs(accel.y),
		abs(accel.z)
	);

	// debug print
	// printfloat3(absAcc);

	if ((absAcc.z > absAcc.x) && (absAcc.z > absAcc.y)) {
		// base orientation on Z
		if (accel.z > 0) {
			orientationString = "board face up";
			orientation = 0;
		}
		else {
			orientationString = "board face down";
			orientation = 1;
		}
	}
	else if ((absAcc.y > absAcc.x) && (absAcc.y > absAcc.z)) {
		// base orientation on Y
		if (accel.y > 0) {
			orientationString = "digital pins up";
			orientation = 2;
		}
		else {
			orientationString = "analog pins up";
			orientation = 3;
		}
	}
	else {
		// base orientation on X
		if (accel.x < 0) {
			orientationString = "connector up";
			orientation = 4;
		}
		else {
			orientationString = "connector down";
			orientation = 5;
		}
	}
}

static void eventCallback(void) {
	if (CurieIMU.getInterruptStatus(CURIE_IMU_ZERO_MOTION)) {
		zeroMotionDetectedMicros = micros();
		//Serial.print("interrupt fired");
		//Serial.println();
	}
}

float3 rotateGravity(const float3 &gravity, const float3 &rot) {
	// TODO
	float3 newGravity = gravity;
	// do operation for x axis
	float3x3 rollMatrix = float3x3RotationX(degToRad(rot.x));
	// do operation for y axis
	float3x3 pitchMatrix = float3x3RotationY(degToRad(rot.y));
	// do operation for z axis
	float3x3 yawMatrix = float3x3RotationZ(degToRad(rot.z));
	newGravity = rollMatrix * newGravity;
	newGravity = pitchMatrix * newGravity;
	newGravity = yawMatrix * newGravity;
	return newGravity;
}

void setGravity() {
	// ** this should only be run if the 
	// ** board is flat and still
	// set gravity (global variable) to the force
	// felt on the axis affected by gravity.
	setOrientation();
	switch(orientation) {
		case 0 : // processor facing up
			gravity = float3(0.0f,0.0f,accel.z);
			break;
		case 1 : // processor facing down
			gravity = float3(0.0f,0.0f,accel.z);
			break;
		case 2 : // digital pins up
			gravity = float3(0.0f,accel.y,0.0f);
			break;
		case 3 : // analog pins up
			gravity = float3(0.0f,accel.y,0.0f);
			break;
		case 4 : // connector up
			gravity = float3(accel.x,0.0f,0.0f);
			break;
		case 5 : // connector down
			gravity = float3(accel.x,0.0f,0.0f);
			break;
		default :
			// TODO - add some form of error catcher
			//Serial.print("orientation wasn't valid");
			//Serial.println();
			break;
	}
	return;
}

bool isZeroMotion() {
	// return true if zero motion was detected within
	// [interval] microseconds from the current time
	// return false otherwise
	microsNow = micros();
	return (abs(microsNow - zeroMotionDetectedMicros) < interval);
}

float convertRawAcceleration(int aRaw) {
	// since we are using Gs
	// -(max range) maps to a raw value of -32768
	// +(max range) maps to a raw value of 32767

	float a = (aRaw * float(CurieIMU.getAccelerometerRange())) / 32768.0;

	return a;
}

float convertRawGyro(int gRaw) {
	// since we are using degrees/seconds
	// -(max range) maps to a raw value of -32768
	// +(max range) maps to a raw value of 32767

	float g = (gRaw * float(CurieIMU.getGyroRange())) / 32768.0;

	return g;
}

float3 convertRawAcceleration(int ax, int ay, int az) {
	float3 modAcc = float3(0.0f);
	modAcc.x = convertRawAcceleration(ax);
	modAcc.y = convertRawAcceleration(ay);
	modAcc.z = convertRawAcceleration(az);
	return modAcc;
}

float3 convertRawGyro(int gx, int gy, int gz) {
	float3 modGyro = float3(0.0f);
	modGyro.x = convertRawGyro(gx);
	modGyro.y = convertRawGyro(gy);
	modGyro.z = convertRawGyro(gz);
	return modGyro;
}

void printFloat3(const float3 &c) {
	Serial.print(c.x);
	Serial.print(",");
	Serial.print(c.y);
	Serial.print(",");
	Serial.print(c.z);
	Serial.println();
}

double radToDeg(double r) {
	double ret = r * 180.0;
	ret /= 3.14;
	return ret;
}

double degToRad(double r) {
	double ret = r / 180.0;
	ret *= 3.14;
	return ret;
}
