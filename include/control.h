#pragma once
#include <Arduino.h>
#include "motors.h"
#include "encoders.h"

struct PID {
  float kp, ki, kd;
  float integral;
  float prevError;
  float maxIntegral;
  float output;
};

void resetPID(PID* pid);
float computePID(PID* pid, float error, float dt);

void updateMotorSpeeds(float dt);
void setMotorSpeedRPM(Motor* motor, Encoder* encoder, PID* pid, float targetRPM);
