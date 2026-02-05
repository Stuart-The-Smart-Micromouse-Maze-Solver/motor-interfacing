#pragma once
#include <Arduino.h>

// Initialize the onboard status LED
void statusLedInit();

// Set LED to green (success/ready)
void statusLedGreen();

// Set LED to red (error/failure)
void statusLedRed();

// Turn off LED
void statusLedOff();
