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
  uint32_t mowUs = micros();
  float dt = (mowUs - lastUs) / 1e6f;
  lastUs = mowUs;

  static bool started = false;
  if (!started) {

  MoveForwardCells(1);
  WaitMs(10000);

  MoveForwardCells(2);
  WaitMs(10000);

  MoveForwardCells(4); 
  WaitMs(10000);

  MoveForwardCells(6);
  WaitMs(10000);
  // TurnRight();
  // WaitMs(150);

  started = true;
  }
  motionUpdate(dt, -1, -1, -1);
}
