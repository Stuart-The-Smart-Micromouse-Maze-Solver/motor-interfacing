#pragma once
#include <Arduino.h>

// Init once in setup
void motionInit();

// Call every loop (executes the active command)
void motionUpdate(float dt, float leftDist=-1, float frontDist=-1, float rightDist=-1);

// Command API (your future algorithm calls these)
bool motionMoveForwardCells(int cells);
bool motionMoveForwardCm(float cm);

bool motionTurnDeg(float deg);   
bool motionTurnLeft90();
bool motionTurnRight90();

// Status
bool motionIsBusy();
void motionStop();
