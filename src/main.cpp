#include <Arduino.h>
#include "motors.h"
#include "encoders.h"
#include "gyro_heading.h"
#include "motion.h"

void setup()
{
  Serial.begin(115200);
  delay(1000);

  motorsInit();
  encodersInit();

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
  delay(2000);
  moveForwardCmClean(18.0f, 200.0f);
  delay(1000);
  turnDegreesGyro(90.0f, 150.0f);
  delay(1000);
  moveBackwardCmClean(2.0f, 200.0f);
  delay(1000);
  moveForwardCmClean(36.0f, 200.0f);
  delay(1000);
  turnDegreesGyro(-90.0f, 150.0f);
  delay(1000);
  moveBackwardCmClean(2.0f, 200.0f);
  delay(1000);
  moveForwardCmClean(18.0f, 200.0f);
  delay(2000);
  turnDegreesGyro(180.0f, 150.0f);
  delay(10000);
}

