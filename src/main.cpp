#include <Arduino.h>
#include "motors.h"
#include "encoders.h"
#include "gyro_heading.h"
#include "motion.h"
#include "config.h"


void setup()
{
  Serial.begin(115200);
  delay(1000);

  motorsInit(); 
  encodersInit();
  //motionInit();

  if (gyroInit()) {
    Serial.println("Gyro OK");
    gyroQuickBiasCal();
  } else {
    Serial.println("Gyro NOT found (heading hold disabled)");
  }

  Serial.println("Init complete");
}

void loop()
{
  static uint32_t lastUs = micros();
  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) / 1e6f;
  lastUs = nowUs;

  motionUpdate(dt, -1, -1, -1);

  enum { START_FWD, WAIT_FWD, START_TURN, WAIT_TURN } static s = START_FWD;

  if (s == START_FWD) {
    if (motionMoveForwardCells(1)) s = WAIT_FWD;
  }
  else if (s == WAIT_FWD) {
    if (!motionIsBusy()) s = START_TURN;
  }
  else if (s == START_TURN) {
    if (motionTurnDeg(90.0f)) s = WAIT_TURN; // right
  }
  else if (s == WAIT_TURN) {
    if (!motionIsBusy()) s = START_FWD;
  }
}
