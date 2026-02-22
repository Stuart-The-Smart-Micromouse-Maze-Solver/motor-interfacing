// #include <Arduino.h>
// #include "config.h"
// #include "motors.h"
// #include "encoders.h"

// const int PWM_FORWARD_MOTOR_LEFT = PWM_CH_L1; 
// const int PWM_BACKWARD_MOTOR_LEFT = PWM_CH_L2;
// const int PWM_FORWARD_MOTOR_RIGHT = PWM_CH_R1;
// const int PWM_BACKWARD_MOTOR_RIGHT = PWM_CH_R2;

// const int MAX_SPEED = 255;
// const int MIN_SPEED = 30; 

// static long lastEncoderLeft = 0;
// static long lastEncoderRight = 0;

// void motorsInit()
// {
//   pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
//   pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

//   pinMode(STBY, OUTPUT);
//   digitalWrite(STBY, HIGH);

//   ledcSetup(PWM_CH_L1, PWM_FREQ, PWM_RESOLUTION);
//   ledcSetup(PWM_CH_L2, PWM_FREQ, PWM_RESOLUTION);
//   ledcSetup(PWM_CH_R1, PWM_FREQ, PWM_RESOLUTION);
//   ledcSetup(PWM_CH_R2, PWM_FREQ, PWM_RESOLUTION);

//   ledcAttachPin(AIN1, PWM_CH_L1);
//   ledcAttachPin(AIN2, PWM_CH_L2);
//   ledcAttachPin(BIN1, PWM_CH_R1);
//   ledcAttachPin(BIN2, PWM_CH_R2);

//   motorsSetLeftPWM(0);
//   motorsSetRightPWM(0);
   
// }

// void motorsSetLeftPWM(int pwm)
//   {
//     if (pwm == 0)
//     {
//       ledcWrite(PWM_FORWARD_MOTOR_LEFT, 0);
//       ledcWrite(PWM_BACKWARD_MOTOR_LEFT, 0);
//     }
//     else if (pwm > 0)
//     {
//       pwm = constrain(pwm, MIN_SPEED, MAX_SPEED);
//       ledcWrite(PWM_FORWARD_MOTOR_LEFT, pwm);
//       ledcWrite(PWM_BACKWARD_MOTOR_LEFT, 0);
//     }
//     else
//     {
//       pwm = constrain(-pwm, MIN_SPEED, MAX_SPEED);
//       ledcWrite(PWM_FORWARD_MOTOR_LEFT, 0);
//       ledcWrite(PWM_BACKWARD_MOTOR_LEFT, -pwm);
//     }
//   }

// void motorsSetRightPWM(int pwm)
//   {
//     if (pwm == 0)
//     {
//       ledcWrite(PWM_FORWARD_MOTOR_RIGHT, 0);
//       ledcWrite(PWM_BACKWARD_MOTOR_RIGHT, 0);
//     }
//     else if (pwm > 0)
//     {
//       pwm = constrain(pwm, MIN_SPEED, MAX_SPEED);
//       ledcWrite(PWM_FORWARD_MOTOR_RIGHT, pwm);
//       ledcWrite(PWM_BACKWARD_MOTOR_RIGHT, 0);
//     }
//     else
//     {
//       pwm = constrain(-pwm, MIN_SPEED, MAX_SPEED);
//       ledcWrite(PWM_FORWARD_MOTOR_RIGHT, 0);
//       ledcWrite(PWM_BACKWARD_MOTOR_RIGHT, -pwm);
//     }
//   }

// void motorsBrakeStop(uint32_t ms)
//   {
//     ledcWrite(PWM_CH_L1, 255); 
//     ledcWrite(PWM_CH_L2, 255);
//     ledcWrite(PWM_CH_R1, 255); 
//     ledcWrite(PWM_CH_R2, 255);
//     delay(ms);
//     ledcWrite(PWM_CH_L1, 0); 
//     ledcWrite(PWM_CH_L2, 0);
//     ledcWrite(PWM_CH_R1, 0); 
//     ledcWrite(PWM_CH_R2, 0);
//   }

// static inline double speedFromEnc(long prevCounts, long currCounts, float dt)
// {
//   long dc = currCounts - prevCounts;
//   return (dc * CM_PER_COUNT) / dt;  // cm/s
// }

// double motorsGetLeftSpeed(float dt)
// {
//   if (dt <= 0.0f) dt = 1e-3f;
//   long curr = (long)readEncoderCounts(leftEncoder);
//   double v = speedFromEnc(lastEncoderLeft, curr, dt);
//   lastEncoderLeft = curr;
//   return v;
// }

// double motorsGetRightSpeed(float dt)
// {
//   if (dt <= 0.0f) dt = 1e-3f;
//   long curr = (long)readEncoderCounts(rightEncoder);
//   double v = speedFromEnc(lastEncoderRight, curr, dt);
//   lastEncoderRight = curr;
//   return v;
// }

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
const int MOTOR_PWM_MIN = 150;
const int MOTOR_PWM_MAX = 255;
const int MOTOR_PWM_RANGE = MOTOR_PWM_MAX - MOTOR_PWM_MIN;

// assumes left and right motors use same PID values
float motor_P = 0.1f;
float motor_I = 0.0f;
float motor_D = 0.0f;
float lastRPMRightRead = 0.0f;
float lastRPMLeftRead = 0.0f;
float readRPMRight() {
  // return 0.0f;
  float rpm = readRPM(rightEncoder);
  Serial.print("\tRPM = ");
  Serial.print(rpm);
  return rpm;
}
float readRPMLeft() {
  // return 0.0f;
  float rpm = readRPM(leftEncoder);
  return rpm;
}
void driveMotorRight(float output) {
  output = constrain(output, -MOTOR_PWM_RANGE, MOTOR_PWM_RANGE);  // constrain it to the "useful" area
  if (output < 0) {
    output -= MOTOR_PWM_MIN;
  }
  else {
    output += MOTOR_PWM_MIN;
  }
  output = constrain(output, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);  // extra constraint for safety...


  Serial.print("\tPWM = ");
  Serial.println((int)output);
  setMotorCommand(&rightMotor, (int)output);
}
void driveMotorLeft(float output) {
  output = constrain(output, -MOTOR_PWM_RANGE, MOTOR_PWM_RANGE);  // constrain it to the "useful" area
  if (output < 0) {
    output -= MOTOR_PWM_MIN;
  }
  else {
    output += MOTOR_PWM_MIN;
  }
  output = constrain(output, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);  // extra constraint for safety...

  setMotorCommand(&leftMotor, output);
}
PIDController<float> rightMotorPID(motor_P, motor_I, motor_D, readRPMRight, driveMotorRight);
PIDController<float> leftMotorPID(motor_P, motor_I, motor_D, readRPMLeft, driveMotorLeft);


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