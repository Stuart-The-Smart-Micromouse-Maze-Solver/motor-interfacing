#pragma once
#include <Arduino.h>

bool gyroInit();
void gyroUpdate();
bool gyroIsValid();
float gyroHeadingDeg();

void gyroResetHeading(float headingDeg = 0.0f);
void gyroQuickBiasCal(uint16_t samples = 400);

// helpers
float angleDiffDeg(float target, float current);
