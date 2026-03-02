#pragma once
#include <Arduino.h>
#include "PID.h"
#include "gyro_heading.h"

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


namespace motors 
{
float readRPMRight();
float readRPMLeft();
float readCountsRight();
float readCountsLeft();
void driveMotorRight(float);
void driveMotorLeft(float);
extern PIDController<float> motorTurnPID;
extern PIDController<float> motorPositionPID;
extern PIDController<float> motorRightVelocityPID;
extern PIDController<float> motorLeftVelocityPID;

void init();
void setCommand(Motor* m, int cmd);
void tick();
void stop();
void brake(uint32_t ms);


}