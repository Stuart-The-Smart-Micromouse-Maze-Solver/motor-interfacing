#pragma once
#include <Arduino.h>
#include "PID.h"
#include "gyro_heading.h"
#include "distance.h"

struct Motor {
  int pinFwd;    // IN1
  int pinRev;    // IN2
  int chFwd;     // PWM channel for IN1
  int chRev;     // PWM channel for IN2
  int command;   // -255..255
  float rpm;
};

extern Motor leftMotor;
extern Motor rightMotor;

void setMotorCommand(Motor* m, int cmd);
void stopMotors();


namespace motors 
{
extern PIDController<float> rotationPID;
extern PIDController<float> positionPID;
extern PIDController<float> rightVelocityPID;
extern PIDController<float> leftVelocityPID;

extern bool isInAction;
extern bool performingTurn;

extern float tof_correction_angle;

extern int32_t cachedLeftCounts;
extern int32_t cachedRightCounts;

void init();
void setCommand(Motor* m, int cmd);
void tick();
void stop();
void brake(uint32_t ms);

void setTargetPosition(float cm);
void setTargetRotation(float deg);

// Zero both position (encoders) and orientation (gyro).
// Aborts any active move, resets encoder counts, resets gyro heading,
// and clears all PID targets and velocity state.
// Call motionAbort() before this if a sequence may be running.
void zero();

void TestTuneInnerControlLoop();

void setTargetRotationCentered(float deg);
void cancelCenteredRotation();
void tickCentered();
}
