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
  autoDemoLoop();
  delay(2000);
}

