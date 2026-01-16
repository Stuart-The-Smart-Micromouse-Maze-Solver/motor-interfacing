#include "config.h"
#include "motors.h"

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

// TB6612 short-brake: IN1=IN2=HIGH (same for B side)
// With PWM pins: just drive both channels at full duty.
void brakeStop(uint32_t ms)
{
  const int FULL = (1 << PWM_RESOLUTION) - 1; // 255 for 8-bit

  ledcWrite(leftMotor.chFwd,  FULL);
  ledcWrite(leftMotor.chRev,  FULL);
  ledcWrite(rightMotor.chFwd, FULL);
  ledcWrite(rightMotor.chRev, FULL);

  delay(ms);
  stopMotors();
}
