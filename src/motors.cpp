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

// SET MOTOR ACTIVE ZONE
const int MOTOR_PWM_MIN = 130;
const int MOTOR_PWM_MAX = 255;
const int MOTOR_PWM_RANGE = MOTOR_PWM_MAX - MOTOR_PWM_MIN;

// assuming: control loop is 100hz = 10ms
// max change in PWM  to prevent slipping?
const int MAX_DELTA_PWM = 10;
// or max change in velocity to prevent slipping
const int MAX_ACCEL = 20; // idk what units



float rightMotorOffset;
float leftMotorOffset;
float rightMotorRPM;
float leftMotorRPM;


// encoder based
float readTurn() {
  
  rightMotorRPM = readRPM(rightEncoder);
  leftMotorRPM = readRPM(leftEncoder);
  // R = readRPM(rightEncoder);
  // L = readRPM(leftEncoder);
  float R = readEncoderCounts(rightEncoder);
  float L = readEncoderCounts(leftEncoder);
  
  float offset = R - L;
  // convert different in encoder ticks into rotation!!!!


  // get absolute adjustment from gyro????
  offset = 0.8 * offset + 0.2 * offset; // replace with gyro
  
  // RANSAC? how would this work lol

  // RANSAC with encoders and gyro into output in degrees


  return offset;
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
  // positive means drive left more (???)
  rightMotorOffset = output;
  leftMotorOffset = -output;
  
  // Serial.print("\t\t2.Mtr.Offset=");
  // Serial.print(output);

  // dont actually drive motors, do that in the position PID loop
}

float readPosition() {
  // float counts = (readEncoderCounts(rightEncoder) + readEncoderCounts(leftEncoder)) / 2;
  float R = readEncoderCounts(rightEncoder);
  float L = readEncoderCounts(leftEncoder);
  Serial.print("\t\tR/L.Pos=");
  Serial.print(R);
  Serial.print("/");
  Serial.print(L);
  
  float counts = (R + L) / 2;

  return counts;
}
void updatePosition(float targetVel) {
  Serial.print("\t\ttargetVel = ");
  Serial.print((int)targetVel);

  // int R = output + rightMotorOffset;
  // if (R < 0) {R -= MOTOR_PWM_MIN;}
  // else if (R > 0) {R += MOTOR_PWM_MIN;}

  // int L = output + leftMotorOffset;
  // if (L < 0) {L -= MOTOR_PWM_MIN;}
  // else if (L > 0) {L += MOTOR_PWM_MIN;}

  // setMotorCommand(&rightMotor, L);
  // setMotorCommand(&leftMotor, R);
  motorRightVelocityPID.setTarget(targetVel);
  motorLeftVelocityPID.setTarget(targetVel);
}

float readRightVelocity() {
  // float avg = readRPM(rightEncoder);
  float avg = rightMotorRPM;  // already retrieved
  Serial.print("\t\tR.Vel = ");
  Serial.print((int)avg);
  Serial.print("/");
  Serial.print((int)motorRightVelocityPID.getTarget());
  return avg;
}
void updateRightVelocity(float pwm) {
  
  Serial.print("\t\tpwm = ");
  Serial.println((int)pwm);

  int R = pwm + rightMotorOffset;
  // if (R < 0) {R -= MOTOR_PWM_MIN;}
  // else if (R > 0) {R += MOTOR_PWM_MIN;}

  // int L = pwm + leftMotorOffset;
  // if (L < 0) {L -= MOTOR_PWM_MIN;}
  // else if (L > 0) {L += MOTOR_PWM_MIN;}

  setMotorCommand(&rightMotor, R);
  // setMotorCommand(&leftMotor, L);
}

float readLeftVelocity() {
  // return readRPM(leftEncoder);
  return leftMotorRPM;  // already retrieved;
}
void updateLeftVelocity(float pwm) {
  setMotorCommand(&leftMotor, pwm + leftMotorOffset);
}

float motor_turn_P = 1.2f;
float motor_turn_I = 0.0f;
float motor_turn_D = 0.0f;

float motor_pos_P = 0.5f;
float motor_pos_I = 0.0f;
float motor_pos_D = 0.0f;

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
  motorTurnPID.setTarget(0);
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