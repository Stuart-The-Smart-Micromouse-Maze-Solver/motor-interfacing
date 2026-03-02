#include "config.h"
#include "motors.h"
#include "Arduino.h"
#include "PID.h"
#include "encoders.h"

Motor leftMotor  = { AIN1, AIN2, PWM_CH_L1, PWM_CH_L2, 0, 0.0f };
Motor rightMotor = { BIN1, BIN2, PWM_CH_R1, PWM_CH_R2, 0, 0.0f };

void motorsInit()
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
}

void setMotorCommand(Motor *m, int cmd)
{
  cmd = constrain(cmd, -255, 255);
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

void brakeMotors(uint32_t ms)
{
  // Short brake: IN1=IN2=HIGH for both motors
  ledcWrite(leftMotor.chFwd, 255);
  ledcWrite(leftMotor.chRev, 255);
  ledcWrite(rightMotor.chFwd, 255);
  ledcWrite(rightMotor.chRev, 255);

  delay(ms);
  stopMotors();
}







namespace motors
{

// set control loop frequencies!!!
const int POSITION_PID_DELAY_MS = 5; // 20ms = 50hz
const int VELOCITY_PID_DELAY_MS = 1;  // 5ms = 200Hz

// SET MOTOR ACTIVE ZONE
const int MOTOR_PWM_MIN = 130;
const int MOTOR_PWM_MAX = 255;
const int MOTOR_PWM_RANGE = MOTOR_PWM_MAX - MOTOR_PWM_MIN;

// assuming: control loop is 200hz = 5ms
// max change in PWM to prevent slipping?
const int MAX_DELTA_PWM = 20;
// or max change in velocity to prevent slipping
const int MAX_ACCEL = 20; // idk what units


uint32_t nowMs;
uint32_t last_pos_pid_tick;
uint32_t last_vel_pid_tick;



float rightMotorRPM;
float leftMotorRPM;

float lastRightPWM;
float lastLeftPWM;

float angularVelOffset;


// encoder based
float readTurn() {
  

  float R = readEncoderCounts(rightEncoder);
  float L = readEncoderCounts(leftEncoder);
  
  // convert encoder ticks to heading
  float encoderDeg = COUNTS_OFFSET_PER_DEG * (R - L);

  // get absolute adjustment from gyro????
  gyroUpdate();
  float gyroDeg = readDeg();
  

  // RANSAC? how would this work lol
  // RANSAC with encoders and gyro into output in degrees
  float alpha = 0.95;

  float fused = (alpha * gyroDeg) + ((1-alpha) * encoderDeg);
  return fused;
}

// gyro based
// float readTurn() {
//   rightMotorRPM = readRPM(rightEncoder);
//   leftMotorRPM = readRPM(leftEncoder);


//   gyroUpdate();
//   float deg = gyroHeadingDeg();
//   Serial.print("\tdeg=");
//   Serial.print(deg);
//   return deg;
// }
void updateTurn(float output) {
  angularVelOffset = output;
}

float readPosition() {
  float R = readEncoderCounts(rightEncoder);
  float L = readEncoderCounts(leftEncoder);
  float counts = (R + L) / 2;

  // float frontDist = 

  return counts;
}
void updatePosition(float targetVel) {
  motorRightVelocityPID.setTarget(targetVel + angularVelOffset);
  motorLeftVelocityPID.setTarget(targetVel - angularVelOffset);
}

float readRightVelocity() {
  return rightMotorRPM;
}
void updateRightVelocity(float pwm) {
  pwm = constrain(pwm, lastRightPWM - MAX_DELTA_PWM, lastRightPWM + MAX_DELTA_PWM);
  setMotorCommand(&rightMotor, pwm);
  lastRightPWM = pwm;
}

float readLeftVelocity() {
  return leftMotorRPM;
}
void updateLeftVelocity(float pwm) {
  pwm = constrain(pwm, lastLeftPWM - MAX_DELTA_PWM, lastLeftPWM + MAX_DELTA_PWM);
  setMotorCommand(&leftMotor, pwm);
  lastLeftPWM = pwm;
}

float motor_pos_P = 0.5f;
float motor_pos_I = 0.0f;
float motor_pos_D = 0.0f;

float motor_turn_P = 0.4f;
float motor_turn_I = 0.0f;
float motor_turn_D = 0.0f;

float motor_vel_P = 0.001f;
float motor_vel_I = 0.05f;
float motor_vel_D = 0.01f;

PIDController<float> motorTurnPID(motor_turn_P, motor_turn_I, motor_turn_D, readTurn, updateTurn);
PIDController<float> motorPositionPID(motor_pos_P, motor_pos_I, motor_pos_D, readPosition, updatePosition);

PIDController<float> motorRightVelocityPID(motor_vel_P, motor_vel_I, motor_vel_D, readRightVelocity, updateRightVelocity);
PIDController<float> motorLeftVelocityPID(motor_vel_P, motor_vel_I, motor_vel_D, readLeftVelocity, updateLeftVelocity);


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
  motorRightVelocityPID.setOutputBounds(-MOTOR_PWM_MAX, MOTOR_PWM_MAX);
  motorLeftVelocityPID.setOutputBounds(-MOTOR_PWM_MAX, MOTOR_PWM_MAX);
  motorTurnPID.setOutputBounds(-MOTOR_PWM_MAX*2, MOTOR_PWM_MAX*2);
  // motorTurnPID.setTarget(0);
}

void setCommand(Motor *m, int cmd)
{
  cmd = constrain(cmd, -255, 255);
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
    
    motorTurnPID.tick();
    motorPositionPID.tick();

  }
  if (nowMs > last_vel_pid_tick + VELOCITY_PID_DELAY_MS) {
    last_vel_pid_tick = nowMs;


    rightMotorRPM = readRPM(rightEncoder);
    leftMotorRPM = readRPM(leftEncoder);

    motorRightVelocityPID.tick();
    motorLeftVelocityPID.tick();


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
}