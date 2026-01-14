#pragma once
#include <Arduino.h>

struct Motor {
  int pwmChannel;
  int pin1;
  int pin2;
  int command; // -255..255
  float rpm;
};

extern Motor leftMotor;
extern Motor rightMotor;

void motorsInit();
void setMotorCommand(Motor* motor, int cmd);
void stopMotors();
void brakeStop(uint32_t ms);
