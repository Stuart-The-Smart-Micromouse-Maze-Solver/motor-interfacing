#ifndef DISTANCE_H
#define DISTANCE_H

#include <Arduino.h>


bool distanceInit();

void distanceUpdateAll();

int getDistanceLeft();

int getDistanceFront();

int getDistanceRight();

float getLateralOffsetMM();

#endif // DISTANCE_H