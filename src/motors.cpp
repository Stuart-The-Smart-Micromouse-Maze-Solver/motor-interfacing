#include "config.h"
#include "motors.h"

Motor leftMotor  = {PWM_CH_LEFT,  AIN1, AIN2, 0, 0};
Motor rightMotor = {PWM_CH_RIGHT, BIN1, BIN2, 0, 0};

void motorsInit()
{
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  ledcSetup(PWM_CH_LEFT,  PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RESOLUTION);

  // Your wiring: PWM on AIN1/BIN1
  ledcAttachPin(AIN1, PWM_CH_LEFT);
  ledcAttachPin(BIN1, PWM_CH_RIGHT);
}

void setMotorCommand(Motor* motor, int cmd)
{
  cmd = constrain(cmd, -255, 255);
  motor->command = cmd;

  if (cmd > 0) {
    digitalWrite(motor->pin2, LOW);
    // PWM is on pin1 (attached), just drive the duty
    ledcWrite(motor->pwmChannel, cmd);
  } else if (cmd < 0) {
    // Reverse with your wiring = “PWM + braking mix”, but works for now
    digitalWrite(motor->pin2, HIGH);
    ledcWrite(motor->pwmChannel, -cmd);
  } else {
    ledcWrite(motor->pwmChannel, 0);
    digitalWrite(motor->pin2, LOW);
  }
}

void stopMotors()
{
  setMotorCommand(&leftMotor, 0);
  setMotorCommand(&rightMotor, 0);
}

void brakeStop(uint32_t ms)
{
  ledcWrite(PWM_CH_LEFT, 255);
  ledcWrite(PWM_CH_RIGHT, 255);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);

  delay(ms);
  stopMotors();
}
