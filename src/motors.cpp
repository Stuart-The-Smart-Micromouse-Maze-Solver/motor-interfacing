#include "config.h"
#include "motors.h"
#include "Arduino.h"
#include "PID.h"
#include "encoders.h"

Motor leftMotor  = { AIN1, AIN2, PWM_CH_L1, PWM_CH_L2, 0, 0.0f };
Motor rightMotor = { BIN1, BIN2, PWM_CH_R1, PWM_CH_R2, 0, 0.0f };

void setMotorCommand(Motor *m, int cmd)
{
  cmd = constrain(cmd, -MOTOR_ACTIVE_PWM_RANGE, MOTOR_ACTIVE_PWM_RANGE);

  if (cmd == 0) {
    m->command = 0;
    ledcWrite(m->chFwd, 0);
    ledcWrite(m->chRev, 0);
    return;
  }

  int pwm = abs(cmd) + MOTOR_PWM_MIN;
  pwm = constrain(pwm, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
  m->command = (cmd > 0) ? pwm : -pwm;

  if (cmd > 0) {
    ledcWrite(m->chFwd, pwm);
    ledcWrite(m->chRev, 0);
  } else {
    ledcWrite(m->chFwd, 0);
    ledcWrite(m->chRev, pwm);
  }
}

void stopMotors()
{
  setMotorCommand(&leftMotor, 0);
  setMotorCommand(&rightMotor, 0);
}

void brakeMotors(uint32_t ms)
{
  ledcWrite(leftMotor.chFwd, 255);
  ledcWrite(leftMotor.chRev, 255);
  ledcWrite(rightMotor.chFwd, 255);
  ledcWrite(rightMotor.chRev, 255);
  delay(ms);
  stopMotors();
}

void brakeStop(uint32_t ms)
{
  brakeMotors(ms);
}

namespace motors {

const int POSITION_PID_DELAY_MS = 10;
const int VELOCITY_PID_DELAY_MS = 10;

const float COMPLETE_POSITION_ERR = 0.5f; // cm
const float COMPLETE_ROTATION_ERR = 1.0f; // deg
const float TURN_DEADBAND_DEG = 1.5f;

const int MAX_ACCEL = 5;

bool isInAction = false;
bool performingTurn = false;

uint32_t nowMs = 0;
uint32_t last_pos_pid_tick = 0;
uint32_t last_vel_pid_tick = 0;
uint32_t actionStartMs = 0;
uint32_t rightStallStartMs = 0;
uint32_t leftStallStartMs = 0;

float lastRightVel = 0.0f;
float lastLeftVel = 0.0f;
float angularVelOffset = 0.0f;
float tof_correction_angle = 0.0f;
float lastRightMeasuredRpm = 0.0f;
float lastLeftMeasuredRpm = 0.0f;

float motor_pos_P = 0.8f;
float motor_pos_I = 0.0f;
float motor_pos_D = 0.0001f;

float motor_turn_P = 1.0f;
float motor_turn_I = 0.0f;
float motor_turn_D = 0.0f;

float motor_vel_P = 0.2f;
float motor_vel_I = 0.0f;
float motor_vel_D = 0.0f;

static float readTurn() {
  float R = readEncoderCounts(rightEncoder);
  float L = readEncoderCounts(leftEncoder);

  float encoderDeg = COUNTS_OFFSET_PER_DEG * (R - L);

  gyroUpdate();
  float gyroDeg = -readDeg();

  const float alpha = 0.98f;
  const float wallGainDegPerMm = 0.05f;

  if (!performingTurn) {
    float distRight = getDistanceRight();
    float distLeft  = getDistanceLeft();

    if (0 < distRight && distRight < 150 && 0 < distLeft && distLeft < 150) {
      float wallDiff = distLeft - distRight;
      tof_correction_angle = constrain(wallDiff * wallGainDegPerMm, -3.0f, 3.0f);
    } else {
      tof_correction_angle = 0.0f;
    }
  } else {
    tof_correction_angle = 0.0f;
  }

  float lateralMM = getLateralOffsetMM();
  tof_correction_angle += constrain(0.02f * lateralMM, -2.0f, 2.0f);

  return alpha * gyroDeg + (1.0f - alpha) * encoderDeg + tof_correction_angle;
}

static void updateTurn(float output) {
  if (abs(rotationPID.getError()) < TURN_DEADBAND_DEG) {
    angularVelOffset = 0.0f;
  } else {
    angularVelOffset = output;
  }
}

// static float readPosition() {
//   float encoderPos = readAvgPosition();
//   int frontMM = getDistanceFront();

//   if (!performingTurn && frontMM > 0 && frontMM < 140) {
//     float targetDistMM = FRONT_TOF_TO_WALL_CM * 10.0f;
//     float errorMM = (float)frontMM - targetDistMM;
//     float tofCorrectionCounts = (errorMM / 10.0f) * COUNTS_PER_CM;
//     return encoderPos + 0.3f * tofCorrectionCounts;
//   }

//   return encoderPos;
// }

float readPosition() {
  return readAvgPosition();
}

static void updatePosition(float targetVel) {
  float currRightVel = targetVel + angularVelOffset;
  float currLeftVel  = targetVel - angularVelOffset;

  currRightVel = constrain(currRightVel, lastRightVel - MAX_ACCEL, lastRightVel + MAX_ACCEL);
  currLeftVel  = constrain(currLeftVel,  lastLeftVel  - MAX_ACCEL, lastLeftVel  + MAX_ACCEL);

  rightVelocityPID.setTarget(currRightVel);
  leftVelocityPID.setTarget(currLeftVel);

  lastRightVel = currRightVel;
  lastLeftVel = currLeftVel;
}

static float readRightVelocity() {
  lastRightMeasuredRpm = readRPM(rightEncoder);
  return lastRightMeasuredRpm;
}

static int applyKickBoost(int rawCmd, float targetRpm, float measuredRpm, uint32_t &stallStartMs) {
  int cmd = rawCmd;
  if (fabsf(targetRpm) > STALL_TARGET_RPM_MIN && fabsf(measuredRpm) < STALL_RPM_THRESHOLD) {
    if (stallStartMs == 0) stallStartMs = millis();
    if ((millis() - stallStartMs) < STALL_KICK_WINDOW_MS) {
      cmd += (targetRpm > 0.0f ? STALL_KICK_BOOST : -STALL_KICK_BOOST);
    }
  } else {
    stallStartMs = 0;
  }
  return cmd;
}

static void updateRightVelocity(float pwm) {
  int cmd = (int)lroundf(pwm);
  cmd = applyKickBoost(cmd, rightVelocityPID.getTarget(), lastRightMeasuredRpm, rightStallStartMs);
  setCommand(&rightMotor, cmd);
}

static float readLeftVelocity() {
  lastLeftMeasuredRpm = readRPM(leftEncoder);
  return lastLeftMeasuredRpm;
}

static void updateLeftVelocity(float pwm) {
  int cmd = (int)lroundf(pwm);
  cmd = applyKickBoost(cmd, leftVelocityPID.getTarget(), lastLeftMeasuredRpm, leftStallStartMs);
  setCommand(&leftMotor, cmd);
}

PIDController<float> rotationPID(motor_turn_P, motor_turn_I, motor_turn_D, readTurn, updateTurn);
PIDController<float> positionPID(motor_pos_P, motor_pos_I, motor_pos_D, readPosition, updatePosition);
PIDController<float> rightVelocityPID(motor_vel_P, motor_vel_I, motor_vel_D, readRightVelocity, updateRightVelocity);
PIDController<float> leftVelocityPID(motor_vel_P, motor_vel_I, motor_vel_D, readLeftVelocity, updateLeftVelocity);

static void resetActionState() {
  angularVelOffset = 0.0f;
  lastRightVel = 0.0f;
  lastLeftVel = 0.0f;
  rightStallStartMs = 0;
  leftStallStartMs = 0;
  lastRightMeasuredRpm = 0.0f;
  lastLeftMeasuredRpm = 0.0f;
  actionStartMs = millis();
}

static void finishAction() {
  isInAction = false;
  angularVelOffset = 0.0f;
  lastRightVel = 0.0f;
  lastLeftVel = 0.0f;
  rightStallStartMs = 0;
  leftStallStartMs = 0;

  rightVelocityPID.setTarget(0.0f);
  leftVelocityPID.setTarget(0.0f);
  rightVelocityPID.setEnabled(false);
  leftVelocityPID.setEnabled(false);

  setCommand(&leftMotor, 0);
  setCommand(&rightMotor, 0);
}

void init()
{
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  ledcSetup(PWM_CH_L1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_L2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_R1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_R2, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(AIN1, PWM_CH_L1);
  ledcAttachPin(AIN2, PWM_CH_L2);
  ledcAttachPin(BIN1, PWM_CH_R1);
  ledcAttachPin(BIN2, PWM_CH_R2);

  stopMotors();

  rightVelocityPID.setOutputBounds(-MOTOR_ACTIVE_PWM_RANGE, MOTOR_ACTIVE_PWM_RANGE);
  leftVelocityPID.setOutputBounds(-MOTOR_ACTIVE_PWM_RANGE, MOTOR_ACTIVE_PWM_RANGE);
  rotationPID.setOutputBounds(-MOTOR_ACTIVE_PWM_RANGE * 2, MOTOR_ACTIVE_PWM_RANGE * 2);

  last_pos_pid_tick = millis();
  last_vel_pid_tick = millis();
}

void setCommand(Motor *m, int cmd)
{
  cmd = constrain(cmd, -MOTOR_ACTIVE_PWM_RANGE, MOTOR_ACTIVE_PWM_RANGE);

  if (abs(cmd) < MOTOR_CMD_DEADBAND) {
    cmd = 0;
  }

  if (cmd == 0) {
    m->command = 0;
    ledcWrite(m->chFwd, 0);
    ledcWrite(m->chRev, 0);
    return;
  }

  int pwmMin = (m == &leftMotor) ? LEFT_MOTOR_PWM_MIN : RIGHT_MOTOR_PWM_MIN;
  int pwm = abs(cmd) + pwmMin;
  pwm = constrain(pwm, pwmMin, MOTOR_PWM_MAX);
  m->command = (cmd > 0) ? pwm : -pwm;

  if (cmd > 0) {
    ledcWrite(m->chFwd, pwm);
    ledcWrite(m->chRev, 0);
  } else {
    ledcWrite(m->chFwd, 0);
    ledcWrite(m->chRev, pwm);
  }
}

void tick() {
  nowMs = millis();

  static uint32_t dbgTimer = 0;
  if (millis() - dbgTimer > 250) {
    dbgTimer = millis();
    Serial.print("[tick] busy=");
    Serial.print(isInAction);
    Serial.print(" posErr=");
    Serial.print(positionPID.getError());
    Serial.print(" posOut=");
    Serial.print(positionPID.getOutput());
    Serial.print(" turnErr=");
    Serial.print(rotationPID.getError());
    Serial.print(" turnOut=");
    Serial.print(rotationPID.getOutput());
    Serial.print(" rVelT=");
    Serial.print(rightVelocityPID.getTarget());
    Serial.print(" lVelT=");
    Serial.print(leftVelocityPID.getTarget());
    Serial.print(" avgPos=");
    Serial.println(readAvgPosition());
  }

  if (!isInAction) {
    return;
  }

  if ((nowMs - actionStartMs) > ACTION_TIMEOUT_MS) {
    finishAction();
    return;
  }

  if ((rightStallStartMs && (nowMs - rightStallStartMs) > STALL_ABORT_MS) ||
      (leftStallStartMs && (nowMs - leftStallStartMs) > STALL_ABORT_MS)) {
    finishAction();
    return;
  }

  if (nowMs - last_pos_pid_tick >= POSITION_PID_DELAY_MS) {
    last_pos_pid_tick = nowMs;

    rotationPID.tick();
    positionPID.tick();

    if (abs(rotationPID.getError()) < COMPLETE_ROTATION_ERR &&
        abs(positionPID.getError()) / COUNTS_PER_CM < COMPLETE_POSITION_ERR) {
      finishAction();
      return;
    }
  }

  if (nowMs - last_vel_pid_tick >= VELOCITY_PID_DELAY_MS) {
    last_vel_pid_tick = nowMs;
    if (rightVelocityPID.isEnabled()) rightVelocityPID.tick();
    if (leftVelocityPID.isEnabled())  leftVelocityPID.tick();
  }
}

void stop() {
  finishAction();
  isInAction = false;
}

void brake(uint32_t ms) {
  brakeMotors(ms);
  stop();
}

void setTargetPosition(float cm) {
  if (isInAction) {
    Serial.println("[motors] setTargetPosition rejected: already busy");
    return;
  }

  Serial.println("[motors] setTargetPosition accepted");
  Serial.print("[motors] cm = ");
  Serial.println(cm);

  performingTurn = false;
  isInAction = true;
  resetActionState();

  resetDeg();
  rotationPID.setTarget(0.0f);

  float currPos = readAvgPosition();
  float target = currPos + cm * COUNTS_PER_CM;

  Serial.print("[motors] currPos = ");
  Serial.println(currPos);
  Serial.print("[motors] targetPos = ");
  Serial.println(target);

  positionPID.setTarget(target);

  rightVelocityPID.setEnabled(true);
  leftVelocityPID.setEnabled(true);
}

void setTargetRotation(float deg) {
  if (isInAction) {
    Serial.println("[motors] setTargetRotation rejected: already busy");
    return;
  }

  Serial.println("[motors] setTargetRotation accepted");
  Serial.print("[motors] deg = ");
  Serial.println(deg);

  performingTurn = true;
  isInAction = true;
  resetActionState();

  positionPID.setTarget(readAvgPosition());

  resetDeg();
  rotationPID.setTarget(deg);

  rightVelocityPID.setEnabled(true);
  leftVelocityPID.setEnabled(true);
}

} // namespace motors
