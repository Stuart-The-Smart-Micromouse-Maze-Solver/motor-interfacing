#pragma once
#include <Arduino.h>

void moveForwardCmClean(float distanceCm, float speedRPM);

void moveBackwardCmClean(float distanceCm, float speedRPM); 

void turnDegreesGyro(float degrees, float speedRPM);

void autoDemoLoop();
