#include "config.h"
#include "control.h"
#include "encoders.h"
#include "motors.h"

static float countsToRPM(int64_t counts, float dt)
{
  if (dt <= 0) return 0;
  float rev = (float)counts / COUNTS_PER_REV;
  return (rev / dt) * 60.0f;
}

void resetPID(PID* pid)
{
  pid->integral = 0;
  pid->prevError = 0;
  pid->output = 0;
}

float computePID(PID* pid, float error, float dt)
{
  if (dt <= 0) dt = 0.001f;

  float p = pid->kp * error;
  pid->integral += error * dt;
  pid->integral = constrain(pid->integral, -pid->maxIntegral, pid->maxIntegral);
  float i = pid->ki * pid->integral;

  float d = pid->kd * (error - pid->prevError) / dt;
  pid->prevError = error;

  pid->output = p + i + d;
  return pid->output;
}

void updateMotorSpeeds(float dt)
{
  int64_t leftNow  = readEncoderCounts(leftEncoder);
  int64_t rightNow = readEncoderCounts(rightEncoder);

  int64_t leftDelta  = leftNow  - leftEncoder.prevCounts;
  int64_t rightDelta = rightNow - rightEncoder.prevCounts;

  leftMotor.rpm  = countsToRPM(leftDelta, dt);
  rightMotor.rpm = countsToRPM(rightDelta, dt);

  leftEncoder.prevCounts  = leftNow;
  rightEncoder.prevCounts = rightNow;
}

void setMotorSpeedRPM(Motor* motor, Encoder* encoder, PID* pid, float targetRPM)
{
  float error = targetRPM - motor->rpm;

  uint32_t now = micros();
  float dt = (now - encoder->prevTime) / 1000000.0f;
  encoder->prevTime = now;

  if (dt > 0.1f) dt = 0.01f;

  float correction = computePID(pid, error, dt);

  int newCmd = motor->command + (int)correction;
  newCmd = constrain(newCmd, -255, 255);

  setMotorCommand(motor, newCmd);
}
