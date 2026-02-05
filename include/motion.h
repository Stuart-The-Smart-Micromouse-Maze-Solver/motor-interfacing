#pragma once
#include <Arduino.h>

// Init once in setup
void motionInit();

// Call every loop (executes the active command)
void motionUpdate(float dt, float leftDist=-1, float frontDist=-1, float rightDist=-1);

// Command API
bool MoveForwardCells(int cells);
bool MoveForwardCm(float cm);
bool TurnRight();   
bool TurnLeft();    
bool Turn180();  
bool WaitMs(uint16_t ms);
bool MaintainDistanceFromWall(float targetDistanceMm, uint32_t durationMs);


// Status
bool motionIsBusy();
void motionStop();
