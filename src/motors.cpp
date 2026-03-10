#include "config.h"
#include "motors.h"
#include "Arduino.h"
#include "PID.h"
#include "encoders.h"

Motor leftMotor  = { AIN1, AIN2, PWM_CH_L1, PWM_CH_L2, 0, 0.0f };
Motor rightMotor = { BIN1, BIN2, PWM_CH_R1, PWM_CH_R2, 0, 0.0f };

// void motorsInit()
// {
//   pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
//   pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

//   pinMode(STBY, OUTPUT);
//   digitalWrite(STBY, HIGH);

//   // Setup PWM channels
//   ledcSetup(PWM_CH_L1, PWM_FREQ, PWM_RESOLUTION);
//   ledcSetup(PWM_CH_L2, PWM_FREQ, PWM_RESOLUTION);
//   ledcSetup(PWM_CH_R1, PWM_FREQ, PWM_RESOLUTION);
//   ledcSetup(PWM_CH_R2, PWM_FREQ, PWM_RESOLUTION);

//   // Attach channels to pins
//   ledcAttachPin(AIN1, PWM_CH_L1);
//   ledcAttachPin(AIN2, PWM_CH_L2);
//   ledcAttachPin(BIN1, PWM_CH_R1);
//   ledcAttachPin(BIN2, PWM_CH_R2);

//   stopMotors();
// }

void setMotorCommand(Motor *m, int cmd)
{
  // remove rough deadzone, makes it less jumpy
  cmd = constrain(cmd, -MOTOR_ACTIVE_PWM_RANGE, MOTOR_ACTIVE_PWM_RANGE);
  if (cmd > 0) {
    cmd += MOTOR_PWM_MIN;
  }
  else {
    cmd -= MOTOR_PWM_MIN;
  }
  m->command = cmd;

  if (cmd > 0) {
    ledcWrite(m->chFwd, cmd);
    ledcWrite(m->chRev, 0);
  } else if (cmd < 0) {
    ledcWrite(m->chFwd, 0);
    ledcWrite(m->chRev, -cmd);
  } else {
    ledcWrite(m->chFwd, 0);
    ledcWrite(m->chRev, 0);
  }
}

void stopMotors()
{
  setMotorCommand(&leftMotor, 0);
  setMotorCommand(&rightMotor, 0);
}

// void brakeMotors(uint32_t ms)
// {
//   // Short brake: IN1=IN2=HIGH for both motors
//   ledcWrite(leftMotor.chFwd, 255);
//   ledcWrite(leftMotor.chRev, 255);
//   ledcWrite(rightMotor.chFwd, 255);
//   ledcWrite(rightMotor.chRev, 255);

//   delay(ms);
//   stopMotors();
// }







namespace motors
{

// set control loop frequencies!!!
// const int POSITION_PID_DELAY_MS = 5;
// const int VELOCITY_PID_DELAY_MS = 1;
const int POSITION_PID_DELAY_MS = 10; // 10ms = 100hz
const int VELOCITY_PID_DELAY_MS = 2;  // 2ms = 500Hz

// SET MOTOR ACTIVE ZONE
// const int MOTOR_PWM_MIN = 100;
// const int MOTOR_PWM_MAX = 255;
// const int MOTOR_ACTIVE_PWM_RANGE = MOTOR_PWM_MAX - MOTOR_PWM_MIN;
// moved to config

// assuming: control loop is 200hz = 5ms
// max change in PWM to prevent slipping?
// const int MAX_DELTA_PWM = 20;
// or max change in velocity to prevent slipping

// const float COMPLETE_ACTION_PERCENT = 0.95; // action will be done at x% of the target
const float COMPLETE_POSITION_ERR = 0.5f; // action is done when within 0.5cm
const float COMPLETE_ROTATION_ERR = 1.0f; // action is done when within 1.0 degree
bool isInAction = false;  // bool to track action


uint32_t nowMs;
uint32_t last_pos_pid_tick;
uint32_t last_vel_pid_tick;



const int MAX_ACCEL = 5; // idk what units or what to even use
float lastRightVel;
float lastLeftVel;


float angularVelOffset;



float readTurn() {
  

  float R = readEncoderCounts(rightEncoder);
  float L = readEncoderCounts(leftEncoder);
  
  // convert encoder ticks to heading
  float encoderDeg = COUNTS_OFFSET_PER_DEG * (R - L);

  // get absolute adjustment from gyro????
  gyroUpdate();
  float gyroDeg = -readDeg(); // yo negative??
  

  // RANSAC? how would this work lol
  // RANSAC with encoders and gyro into output in degrees
  float alpha = 0.95;

  float fused = (alpha * gyroDeg) + ((1-alpha) * encoderDeg);



  // also read L and R ToF sensors if enabled
  // during turns it shouldnt be enabled, going straights it should be enabled
  /*
  if (enableSideTOFTracking) {
    // offset_translation = ((readLeftTOF() - SIDE_TOF_TO_WALL) % 18.0f + (readRightTOF() - SIDE_TOF_TO_WALL) % 18.0f) / 2;
    // somehow adjust fused based on offset_translation
  }
  
  */

  return fused;
}
void updateTurn(float output) {
  angularVelOffset = output;
}

float readPosition() {
  // float R = readEncoderCounts(rightEncoder);
  // float L = readEncoderCounts(leftEncoder);
  // float counts = (R + L) / 2;
  float counts = readAvgPosition();

  // float frontDist = 

  // FUSE ENCODER DATA WITH FRONT SENSOR DATA

  return counts;
}
void updatePosition(float targetVel) {
  float currRightVel = targetVel + angularVelOffset;
  float currLeftVel = targetVel - angularVelOffset;

  // clamp to limit accel
  currRightVel = constrain(currRightVel, lastRightVel-MAX_ACCEL, lastRightVel+MAX_ACCEL);
  currLeftVel = constrain(currLeftVel, lastLeftVel-MAX_ACCEL, lastLeftVel+MAX_ACCEL);


  rightVelocityPID.setTarget(targetVel + angularVelOffset);
  leftVelocityPID.setTarget(targetVel - angularVelOffset);

  lastRightVel = currRightVel;
  lastLeftVel = currLeftVel;
}

float readRightVelocity() {
  // return rightMotorRPM;
  return readRPM(rightEncoder);
}
void updateRightVelocity(float pwm) {
  // pwm = constrain(pwm, lastRightPWM - MAX_DELTA_PWM, lastRightPWM + MAX_DELTA_PWM); // idk if this is the proper way to limit accel
  setCommand(&rightMotor, pwm);
  // lastRightPWM = pwm;
}

float readLeftVelocity() {
  // return leftMotorRPM;
  return readRPM(leftEncoder);
}
void updateLeftVelocity(float pwm) {
  // pwm = constrain(pwm, lastLeftPWM - MAX_DELTA_PWM, lastLeftPWM + MAX_DELTA_PWM); // idk if this is the proper way to limit accel
  setCommand(&leftMotor, pwm);
  // lastLeftPWM = pwm;
}


float motor_pos_P = 0.8f;     //0.6
float motor_pos_I = 0.0f; //0.0
float motor_pos_D = 0.0001f;

float motor_turn_P = 1.0f;    //0.8
float motor_turn_I = 0.0f;    //0.0
float motor_turn_D = 0.0f;

float motor_vel_P = 0.2f;    // 0.04
float motor_vel_I = 0.04f;     // 0.04? currently too slow so think you need these or higher
float motor_vel_D = 0.0f;



PIDController<float> rotationPID(motor_turn_P, motor_turn_I, motor_turn_D, readTurn, updateTurn);
PIDController<float> positionPID(motor_pos_P, motor_pos_I, motor_pos_D, readPosition, updatePosition);

PIDController<float> rightVelocityPID(motor_vel_P, motor_vel_I, motor_vel_D, readRightVelocity, updateRightVelocity);
PIDController<float> leftVelocityPID(motor_vel_P, motor_vel_I, motor_vel_D, readLeftVelocity, updateLeftVelocity);


/*
Encoder Count   ->  Position PID (Target: distance)  -> TargetVel
Encoder RPM     ->  Velocity PID (Target: TargetVel)  -> TargetPWM

*/

void init()
{
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Setup PWM channels
  ledcSetup(PWM_CH_L1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_L2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_R1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_R2, PWM_FREQ, PWM_RESOLUTION);

  // Attach channels to pins
  ledcAttachPin(AIN1, PWM_CH_L1);
  ledcAttachPin(AIN2, PWM_CH_L2);
  ledcAttachPin(BIN1, PWM_CH_R1);
  ledcAttachPin(BIN2, PWM_CH_R2);

  stopMotors();


  // motorPositionPID.setOutputBounds(-MOTOR_PWM_MAX, MOTOR_PWM_MAX);  // outputs velocity..? no need for bounds? or what
  // motorVelocityPID.setOutputBounds(-MOTOR_PWM_RANGE, MOTOR_PWM_RANGE);
  // rightVelocityPID.setOutputBounds(-MOTOR_PWM_MAX, MOTOR_PWM_MAX);
  // leftVelocityPID.setOutputBounds(-MOTOR_PWM_MAX, MOTOR_PWM_MAX);
  // rotationPID.setOutputBounds(-MOTOR_PWM_MAX*2, MOTOR_PWM_MAX*2);

  // with deadzone
  rightVelocityPID.setOutputBounds(-MOTOR_ACTIVE_PWM_RANGE, MOTOR_ACTIVE_PWM_RANGE);
  leftVelocityPID.setOutputBounds(-MOTOR_ACTIVE_PWM_RANGE, MOTOR_ACTIVE_PWM_RANGE);
  rotationPID.setOutputBounds(-MOTOR_ACTIVE_PWM_RANGE*2, MOTOR_ACTIVE_PWM_RANGE*2);
  
}

void setCommand(Motor *m, int cmd)
{
  // remove rough deadzone, makes it less jumpy
  cmd = constrain(cmd, -MOTOR_ACTIVE_PWM_RANGE, MOTOR_ACTIVE_PWM_RANGE);
  if (cmd > 0) {
    cmd += MOTOR_PWM_MIN;
  }
  else {
    cmd -= MOTOR_PWM_MIN;
  }
  m->command = cmd;

  if (cmd > 0) {
    ledcWrite(m->chFwd, cmd);
    ledcWrite(m->chRev, 0);
  } else if (cmd < 0) {
    ledcWrite(m->chFwd, 0);
    ledcWrite(m->chRev, -cmd);
  } else {
    ledcWrite(m->chFwd, 0);
    ledcWrite(m->chRev, 0);
  }
}

void tick() {
  nowMs = millis();
  if (nowMs > last_pos_pid_tick + POSITION_PID_DELAY_MS) {
    last_pos_pid_tick = nowMs;
    
    // tick PID controls
    rotationPID.tick(); // in this order
    positionPID.tick();

    // check if targets are if within the done area
    if (abs(rotationPID.getError()) < COMPLETE_ROTATION_ERR &&
        abs(positionPID.getError()) / COUNTS_PER_CM < COMPLETE_POSITION_ERR) {
      // action is done!
      isInAction = false;
    }

  }
  if (nowMs > last_vel_pid_tick + VELOCITY_PID_DELAY_MS) {
    last_vel_pid_tick = nowMs;

    // rightMotorRPM = readRPM(rightEncoder);
    // leftMotorRPM = readRPM(leftEncoder);
    
    // tick PID controls
    rightVelocityPID.tick();
    leftVelocityPID.tick();


  }
}

void stop()
{
  setCommand(&leftMotor, 0);
  setCommand(&rightMotor, 0);
}

void brake(uint32_t ms)
{
  // Short brake: IN1=IN2=HIGH for both motors
  ledcWrite(leftMotor.chFwd, 255);
  ledcWrite(leftMotor.chRev, 255);
  ledcWrite(rightMotor.chFwd, 255);
  ledcWrite(rightMotor.chRev, 255);

  delay(ms);
  stopMotors();
}


void setTargetPosition(float cm) {
  if (isInAction) return;
  isInAction = true;

  positionPID.setTarget(readAvgPosition() + cm * COUNTS_PER_CM);
}

void setTargetRotation(float deg) {
  if (isInAction) return;
  isInAction = true;
  
  float currentHeading = readDeg();
  rotationPID.setTarget(currentHeading + deg);

}

}//namespace motors