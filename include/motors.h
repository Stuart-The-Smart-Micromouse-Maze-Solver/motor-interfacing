#pragma once
#include <Arduino.h>

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

void motorsInit();
void setMotorCommand(Motor* m, int cmd);
void stopMotors();
void brakeStop(uint32_t ms);
