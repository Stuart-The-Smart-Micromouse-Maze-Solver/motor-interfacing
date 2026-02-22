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
    // MoveForwardCells(1);
    // WaitMs(500);

    // TurnRight();
    // WaitMs(150);

    uint32_t temp_timer_outer = millis();
    uint32_t last_PID_tick = millis();
    
    Serial.println("START RUNNING 30 RPM");
    motors::rightMotorPID.setTarget(30);  // set target 30 rpm
    while (nowMs < temp_timer_outer + 5000) { // try to run for 5 sec
      nowMs = millis();

      if (nowMs > last_PID_tick + 10) { // 10ms = 100Hz
        last_PID_tick = nowMs;
        motors::rightMotorPID.tick();
      }
      
    }

    Serial.println("START RUNNING 60 RPM");
    motors::rightMotorPID.setTarget(60);  // set target 30 rpm
    while (nowMs < temp_timer_outer + 5000) { // try to run for 5 sec
      nowMs = millis();

      if (nowMs > last_PID_tick + 10) { // 10ms = 100Hz
        last_PID_tick = nowMs;
        motors::rightMotorPID.tick();
      }
      
    }

    stopMotors();


    started = true;
  }
  motionUpdate(dt, -1, -1, -1);
}
