#pragma once
#include <Arduino.h>

bool gyroInit();
void gyroQuickBiasCal(uint16_t samples = 400);
void gyroUpdate();


// delete:
bool gyroIsValid();
float gyroHeadingDeg();
void gyroResetHeading(float headingDeg = 0.0f);

// helpers
float angleDiffDeg(float target, float current);

float readDeg();
void resetDeg();
