#pragma once
#include <Arduino.h>

// void motorsInit();

// void motorsSetLeftPWM(int pwm);
// void motorsSetRightPWM(int pwm);
// void motorsBrakeStop(uint32_t ms);

// // Optional (but useful for your PID)
// double motorsGetLeftSpeed(float dt);
// double motorsGetRightSpeed(float dt);

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
void brakeMotors(uint32_t ms);
//void brakeStop(uint32_t ms);
