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

  // motorsInit(); 
  motors::init();
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
  uint32_t nowMs = millis();

  static bool started = false;
  if (!started) {
    // MoveForwardCells(1);
    // WaitMs(500);

    // TurnRight();
    // WaitMs(150);

    uint32_t temp_timer_outer = millis();
    uint32_t last_PID_tick = millis();
    


    // TESTING TARGET ENCODER COUNTS
    Serial.println("START RUNNING 1 CELL (243 counts)");
    motors::motorPositionPID.setTarget(readEncoderCounts(rightEncoder) + COUNTS_PER_CELL);
    motors::motorTurnPID.setTarget(0);  // straight
    while (nowMs < temp_timer_outer + 5000) { // try to run for 5 sec
      nowMs = millis();

      if (nowMs > last_PID_tick + 10) { // 10ms = 100Hz
        last_PID_tick = nowMs;
        motors::motorTurnPID.tick();  // call turn before position
        motors::motorPositionPID.tick();
      }
      
    }

    resetEncoderCounts();
    temp_timer_outer = millis();
    Serial.println("START RUNNING 3 CELL (729 counts)");
    motors::motorPositionPID.setTarget(readEncoderCounts(rightEncoder) + 3 * COUNTS_PER_CELL);
    motors::motorTurnPID.setTarget(0);  // straight
    while (nowMs < temp_timer_outer + 5000) { // try to run for 5 sec
      nowMs = millis();

      if (nowMs > last_PID_tick + 10) { // 10ms = 100Hz
        last_PID_tick = nowMs;
        motors::motorTurnPID.tick();  // call turn before position
        motors::motorPositionPID.tick();
      }
      
    }

    Serial.println("Stopping test");
    stopMotors();

    // while (true) {
    //   Serial.println(readEncoderCounts(rightEncoder));
    // }

    started = true;
  }
  motionUpdate(dt, -1, -1, -1);
}
