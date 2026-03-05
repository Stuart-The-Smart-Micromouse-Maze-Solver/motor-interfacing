#include <Arduino.h>
#include "motors.h"
#include "encoders.h"
#include "gyro_heading.h"
#include "motion.h"
#include "config.h"
#include "distance.h"
#include <Wire.h>

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // // motorsInit(); 
  // motors::init();
  // encodersInit();
  // //motionInit();

    Serial.println("Init complete");

  if (distanceInit()) {
    Serial.println("Distance Sensors: OK");
  } else {
    Serial.println("Distance Sensors: FAILED (Check wiring/XSHUT)");
  }

  if (gyroInit()) {
    Serial.println("Gyro OK");
    gyroQuickBiasCal();
  } else {
    Serial.println("Gyro NOT found (heading hold disabled)");
  }


}

void loop()
{
  // --- 3. SENSOR UPDATES ---
  static uint32_t lastUs = micros();
  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) / 1e6f;
  lastUs = nowUs;
  uint32_t nowMs = millis();

  // Continuously poll ToF sensors
  distanceUpdateAll();
  gyroUpdate(); 

  // --- 4. DEBUG PRINT (Every 100ms) ---
  static uint32_t lastPrint = 0;
  if (nowMs - lastPrint > 100) {
    lastPrint = nowMs;
    Serial.printf("L: %4d | C: %4d | R: %4d mm\n", 
                  getDistanceLeft(), getDistanceFront(), getDistanceRight());
    Serial.printf("Heading: %6.2f deg\n", gyroHeadingDeg());
    Serial.println();
  }
  
}
