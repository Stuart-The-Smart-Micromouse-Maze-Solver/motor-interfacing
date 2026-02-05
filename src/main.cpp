#include <Arduino.h>
#include "motors.h"
#include "encoders.h"
#include "gyro_heading.h"
#include "motion.h"
#include "config.h"
#include "distance.h"
#include "status_led.h"
#include <Wire.h>


void setup()
{
  Serial.begin(115200);
  delay(1000);

  // Initialize status LED
  statusLedInit();
  statusLedOff();

  // Initialize I2C bus
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C

  motorsInit(); 
  encodersInit();
  //motionInit();

  if (gyroInit()) {
    Serial.println("Gyro OK");
    gyroQuickBiasCal();
  } else {
    Serial.println("Gyro NOT found (heading hold disabled)");
  }

  // Initialize distance sensor
  if (distanceInit()) {
    Serial.println("Distance sensor OK");
  } else {
    Serial.println("Distance sensor NOT found");
  }

  Serial.println("Init complete");
}

void loop()
{
  static uint32_t lastUs = micros();
  uint32_t mowUs = micros();
  float dt = (mowUs - lastUs) / 1e6f;
  lastUs = mowUs;

  // Get distance reading from front sensor
  int frontDist = getDistanceFront();
  if (frontDist > 0) {
    statusLedGreen();
  }
  static bool started = false;
  if (!started) {    
    // Maintain 100mm distance from wall for 10 seconds
    MaintainDistanceFromWall(100.0f, 1000000);
    
    started = true;
  }
  
  // Update motion with front distance sensor reading
  motionUpdate(dt, -1, (float)frontDist, -1);
}
