#ifndef DISTANCE_H
#define DISTANCE_H

#include <Arduino.h>


bool distanceInit();

void distanceUpdateAll();

int getDistanceLeft();

int getDistanceFront();

// Latest unfiltered front reading — use for collision abort to bypass EMA lag.
int getDistanceFrontRaw();

int getDistanceRight();

#endif // DISTANCE_H