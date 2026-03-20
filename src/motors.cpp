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
const int POSITION_PID_DELAY_US = 5000;  // 5ms = 200Hz
const int VELOCITY_PID_DELAY_US = 1000;  // 1ms = 1kHz

// SET MOTOR ACTIVE ZONE
// const int MOTOR_PWM_MIN = 100;
// const int MOTOR_PWM_MAX = 255;
// const int MOTOR_ACTIVE_PWM_RANGE = MOTOR_PWM_MAX - MOTOR_PWM_MIN;
// moved to config

// assuming: control loop is 200hz = 5ms
// max change in PWM to prevent slipping?
// const int MAX_DELTA_PWM = 20;
// or max change in velocity to prevent slipping

const float COMPLETE_POSITION_ERR = 0.5f; // action is done when within 0.5cm         // UPDATE THIS FOR FINAL VERSION
const float COMPLETE_ROTATION_ERR = 3.0f; // action is done when within 1.0 degree    // UPDATE THIS FOR FINAL VERSION
bool isInAction = false;  // bool to track action
bool performingTurn;

uint32_t nowUs;
uint32_t last_pos_pid_tick;
uint32_t last_vel_pid_tick;

// Encoder counts cached once per position PID tick to avoid multiple critical sections
static int32_t cachedLeftCounts  = 0;
static int32_t cachedRightCounts = 0;




float lastRightVel;
float lastLeftVel;


float angularVelOffset;

float tof_correction_angle;  // made extern float TEMPORARILY

float readTurn() {
  float R = (float)cachedRightCounts;
  float L = (float)cachedLeftCounts;
  
  // convert encoder ticks to heading
  float encoderDeg = COUNTS_OFFSET_PER_DEG * (R - L);

  // get absolute adjustment from gyro????
  gyroUpdate();
  float gyroDeg = -readDeg(); // yo negative??
  

  // RANSAC? how would this work lol
  // RANSAC with encoders and gyro into output in degrees
  // float alpha = 0.95;
  float alpha = 1.0f;
  
  
  
  // also read L and R ToF sensors if enabled
  // during turns it shouldnt be enabled, going straights it should be enabled
  const float k_wall = 0.03f;  // adjust based on how much angle correction based on ToF reading
  // float tof_correction_angle;  // made extern float TEMPORARILY

  // if (enableSideTOFTracking) {
  if (!performingTurn) {
    // offset_translation = ((readLeftTOF() - SIDE_TOF_TO_WALL) % 18.0f + (readRightTOF() - SIDE_TOF_TO_WALL) % 18.0f) / 2;
    // somehow adjust fused based on offset_translation
    float distRight = getDistanceRight(); // mm
    float distLeft = getDistanceLeft(); // mm
    
    if (0 < distRight && distRight < 150 && 0 < distLeft && distLeft < 150) {
      tof_correction_angle = k_wall * -(distRight - distLeft);
    }
    else {
      tof_correction_angle = 0;
    }
  }
  else {
    tof_correction_angle = 0;
  }
    
    float fused = (alpha * gyroDeg) + ((1-alpha) * encoderDeg) + tof_correction_angle;

  float target = rotationPID.getTarget();
  if (fabsf(fused - target) < 0.5f) return target;

  return fused;
}
void updateTurn(float output) {
  static float smoothed = 0.0f;
  smoothed = 0.3f * output + 0.7f * smoothed;  // alpha=0.3 (~5 tick time constant)
  angularVelOffset = smoothed;
}

float readPosition() {
  float encoderPos = (cachedRightCounts + cachedLeftCounts) / 2.0f;

  // float frontDist = 

  // FUSE ENCODER DATA WITH FRONT SENSOR DATA

  int frontMM = getDistanceFront();

  /*
  // Only trust front ToF when it's reading a close, valid wall
  if (frontMM > 0 && frontMM < 150) {
      // Convert mm to encoder counts equivalent
      // Target: robot should be FRONT_TOF_TO_WALL_CM * 10 mm from front wall
      float distFromWallMM = (float)frontMM;
      float targetDistMM   = FRONT_TOF_TO_WALL_CM * 10.0f;
      float errorMM        = distFromWallMM - targetDistMM;

      // Blend: trust ToF more the closer and more stable it is
      // Convert mm error to encoder counts and apply a soft correction
      float tofCorrectionCounts = (errorMM / 10.0f) * COUNTS_PER_CM; // mm->cm->counts
      return encoderPos + tofCorrectionCounts * 0.5f; // trust ToF 0.5
  }*/

  return encoderPos;
}
void updatePosition(float targetVel) {
  float currRightVel = targetVel + angularVelOffset;
  float currLeftVel = targetVel - angularVelOffset;

  // clamp to limit accel
  currRightVel = constrain(currRightVel, lastRightVel-MAX_ACCEL, lastRightVel+MAX_ACCEL);
  currLeftVel = constrain(currLeftVel, lastLeftVel-MAX_ACCEL, lastLeftVel+MAX_ACCEL);


  rightVelocityPID.setTarget(currRightVel);
  leftVelocityPID.setTarget(currLeftVel);

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


float motor_pos_P = 0.9f;
float motor_pos_I = 0.001f;
float motor_pos_D = 0.5f;

float motor_turn_P = 1.8f;
float motor_turn_I = 0.0004f;
float motor_turn_D = 1.0f;

float motor_vel_P = 1.2f;
float motor_vel_I = 0.0f;
float motor_vel_D = 0.1f;



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


  positionPID.registerTimeFunction(micros);
  rotationPID.registerTimeFunction(micros);
  rightVelocityPID.registerTimeFunction(micros);
  leftVelocityPID.registerTimeFunction(micros);

  // with deadzone
  rightVelocityPID.setOutputBounds(-MOTOR_ACTIVE_PWM_RANGE, MOTOR_ACTIVE_PWM_RANGE);
  leftVelocityPID.setOutputBounds(-MOTOR_ACTIVE_PWM_RANGE, MOTOR_ACTIVE_PWM_RANGE);
  rotationPID.setOutputBounds(-MOTOR_ACTIVE_PWM_RANGE*2, MOTOR_ACTIVE_PWM_RANGE*2);
  
}

void setCommand(Motor *m, int cmd)
{
  // remove rough deadzone, makes it less jumpy
  cmd = constrain(cmd, -MOTOR_ACTIVE_PWM_RANGE, MOTOR_ACTIVE_PWM_RANGE);

  // command of 0 catch
  if (cmd == 0){
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
  } 
  else {
    ledcWrite(m->chFwd, 0);
    ledcWrite(m->chRev, pwm);
  }
}

void tick() {
  nowUs = micros();


  if (nowUs > last_pos_pid_tick + POSITION_PID_DELAY_US) {
    last_pos_pid_tick = nowUs;

    // Single atomic read of both encoders for entire position tick
    readBothEncoders(cachedLeftCounts, cachedRightCounts);

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
  if (nowUs > last_vel_pid_tick + VELOCITY_PID_DELAY_US) {
    last_vel_pid_tick = nowUs;

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
  
  resetDeg();
  // float currentHeading = readDeg();

  // rotationPID.setTarget(currentHeading + deg);
  rotationPID.setTarget(deg);

}

}//namespace motors