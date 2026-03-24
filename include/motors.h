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

void init();
void setCommand(Motor* m, int cmd);
void tick();
void stop();
void brake(uint32_t ms);

void setTargetPosition(float cm);
void setTargetRotation(float deg);

void TestTuneInnerControlLoop();
}
