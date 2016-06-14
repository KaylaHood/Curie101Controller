#pragma once

void updateValues(bool printingEnabled, bool detectZeroMotionEnabled);

void calibrate();

void setOrientation();

static void eventCallback(void);

float3 rotateGravity(const float3 &gravity, const float3 &rot);

void setGravity();

bool isZeroMotion();

float convertRawAcceleration(int aRaw);

float convertRawGyro(int gRaw);

float3 convertRawAcceleration(int ax, int ay, int az);

float3 convertRawGyro(int gx, int gy, int gz);

void printFloat3(const float3 &c);

double radToDeg(double r);

double degToRad(double r);
