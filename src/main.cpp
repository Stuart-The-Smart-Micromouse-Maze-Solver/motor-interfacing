#include <Arduino.h>
#include "motors.h"
#include "encoders.h"
#include "gyro_heading.h"
#include "motion.h"
#include "config.h"
#include "server.h"


RobotServer robotServer(
  &motors::motorTurnPID,
  &motors::motorPositionPID,
  &motors::motorRightVelocityPID,
  &motors::motorLeftVelocityPID
);
volatile bool needsRestart = false;

void triggerRestart() {
  needsRestart = true;
}
void setTargetPosition(int cellCount) {
  motors::motorPositionPID.setTarget((readEncoderCounts(rightEncoder) + readEncoderCounts(leftEncoder))/2 + cellCount * COUNTS_PER_CELL);
}
void setTargetTurn(float deg) {
  motors::motorTurnPID.setTarget(deg);
}



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

  // start webserver
  // robotServer.begin(&motors::motorTurnPID, &motors::motorPositionPID, &motors::motorLeftVelocityPID, &motors::motorRightVelocityPID, temp_restart_func);
  // robotServer.log("WEBSERVER INITIALIZED...");
  Serial.println("Init complete");


  xTaskCreatePinnedToCore(
    [](void* p){ 
        robotServer.begin(
          "MINECRAFT_WIFI", 
          "ieeeieee", 
          triggerRestart,
          setTargetPosition,
          setTargetTurn
        );
        for(;;) { vTaskDelay(1000 / portTICK_PERIOD_MS); } // Keep task alive
    },
    "WebServerTask",
    8192,  // Stack size
    NULL,
    1,     // Priority
    NULL,
    0      // Core 0
  );
}

void loop()
{
  static uint32_t lastUs = micros();
  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) / 1e6f;
  lastUs = nowUs;
  uint32_t nowMs = millis();

  uint32_t temp_timer_outer = millis();
  uint32_t last_PID_tick = millis();
  

  /*  // TESTING TARGET ENCODER COUNTS
  Serial.println("START RUNNING 1 CELL (243 counts)");
  motors::motorPositionPID.setTarget((readEncoderCounts(rightEncoder) + readEncoderCounts(leftEncoder))/2 + COUNTS_PER_CELL);
  motors::motorTurnPID.setTarget(0);  // straight
  while (nowMs < temp_timer_outer + 6000) { // try to run for 5 sec
    nowMs = millis();

    if (nowMs > last_PID_tick + 10) { // 10ms = 100Hz
      last_PID_tick = nowMs;
      motors::motorTurnPID.tick();  // call in this order!! to be updated with a single function
      motors::motorPositionPID.tick();
      motors::motorRightVelocityPID.tick();
      motors::motorLeftVelocityPID.tick();

      // also run in loop lol
      // if (random(0, 10) >= 8) {
      //   robotServer.sendTelemetry(random(0, 100) / 10.0, 20.0, 30.0);
      // }
      if (needsRestart) break;
    }
    
  }

  // resetEncoderCounts();
  temp_timer_outer = millis();
  Serial.println("START RUNNING 3 CELL (729 counts)");
  motors::motorPositionPID.setTarget((readEncoderCounts(rightEncoder) + readEncoderCounts(leftEncoder))/2 + 3 * COUNTS_PER_CELL);
  motors::motorTurnPID.setTarget(0);  // straight
  while (nowMs < temp_timer_outer + 10000) { // try to run for 5 sec
    nowMs = millis();

    if (nowMs > last_PID_tick + 10) { // 10ms = 100Hz
      last_PID_tick = nowMs;
      motors::motorTurnPID.tick();  // call in this order!! to be updated with a single function
      motors::motorPositionPID.tick();
      motors::motorRightVelocityPID.tick();
      motors::motorLeftVelocityPID.tick();

      // also run in loop lol
      // if (random(0, 10) >= 8) {
      //   robotServer.sendTelemetry(random(0, 100) / 10.0, 20.0, 30.0);
      // }

      if (needsRestart) break;
    }
    
  }
  */


  while (!needsRestart) {
    motors::tick();
  }
  needsRestart = false;
}
