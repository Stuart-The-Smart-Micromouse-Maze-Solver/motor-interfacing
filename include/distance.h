#pragma once
#include <Arduino.h>

// Initialize the front distance sensor
bool distanceInit();

// Get distance reading in mm
int getDistanceFront();

// Check if sensor has new data ready
bool distanceDataReady();
