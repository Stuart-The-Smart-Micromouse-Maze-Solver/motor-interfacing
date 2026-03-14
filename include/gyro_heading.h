#pragma once
#include <Arduino.h>

bool gyroInit();
void gyroQuickBiasCal(uint16_t samples = 400);
void gyroCache();   // blocking I2C read — call from Core 0 task
void gyroUpdate();  // lightweight integration — call from control loop


// delete:
bool gyroIsValid();
float gyroHeadingDeg();
void gyroResetHeading(float headingDeg = 0.0f);

// helpers
float angleDiffDeg(float target, float current);

float readDeg();
void resetDeg();
