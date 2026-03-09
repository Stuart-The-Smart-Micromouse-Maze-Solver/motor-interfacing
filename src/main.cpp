#include <Arduino.h>
#include "motors.h"
#include "encoders.h"
#include "gyro_heading.h"
#include "motion.h"
#include "config.h"
#include "server.h"


RobotServer robotServer(
  &motors::rotationPID,
  &motors::positionPID,
  &motors::rightVelocityPID,
  &motors::leftVelocityPID
);
volatile bool needsRestart = false;


void triggerStart() {
  motors::rightVelocityPID.setEnabled(true);
  motors::leftVelocityPID.setEnabled(true);
}
void triggerStop() {
  motors::rightVelocityPID.setEnabled(false);
  motors::leftVelocityPID.setEnabled(false);
  motors::stop();
}
void triggerRestart() {
  needsRestart = true;
}
void setTargetPosition(int cellCount) {
  // motors::motorPositionPID.setTarget((readEncoderCounts(rightEncoder) + readEncoderCounts(leftEncoder))/2 + cellCount * COUNTS_PER_CELL);
  motors::setTargetPosition(cellCount * 18.0f);
}
void setTargetTurn(float deg) {
  // motors::motorTurnPID.setTarget(deg);
  motors::setTargetRotation(deg);
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
    gyroQuickBiasCal(4000); // pre-bake the offset? so boot is faster?
    Serial.println("Gyro OK");
  } else {
    Serial.println("Gyro NOT found (heading hold disabled)");
  }



  xTaskCreatePinnedToCore(
    [](void* p){ 
        robotServer.begin(
          "MINECRAFT_WIFI", 
          "ieeeieee", 
          triggerStart,
          triggerStop,
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

  /*

  // general workflow
  robotServer.log("1. Going straight...");
  motors::setTargetPosition(18);
  while (motors::isInAction) {motors::tick();}
  
  robotServer.log("2. Turning Right...");
  motors::setTargetRotation(-90);
  while (motors::isInAction) {motors::tick();}

  robotServer.log("3. Going straight...");
  motors::setTargetPosition(18);
  while (motors::isInAction) {motors::tick();}

  robotServer.log("4. Turning left...");
  motors::setTargetRotation(90);
  while (motors::isInAction) {motors::tick();}
  */

  while (!needsRestart) {
    nowMs = millis();

    motors::tick();

    // robotServer.log("Gyro IMU: " + String(gyroHeadingDeg()));
    // robotServer.log("Gyro ENC: " + String(COUNTS_OFFSET_PER_DEG * (readEncoderCounts(rightEncoder) - readEncoderCounts(leftEncoder))));
    if (nowMs > temp_timer_outer + 100) {
      // log 10hz
      temp_timer_outer = nowMs;
      // robotServer.log("Gyro IMU/ENC: " + String(readDeg()) + " / " + String(COUNTS_OFFSET_PER_DEG * (readEncoderCounts(rightEncoder) - readEncoderCounts(leftEncoder))));
    }
  }
  needsRestart = false;

  
  robotServer.log("1. Going straight...");
  motors::setTargetPosition(18);
  while (motors::isInAction) {motors::tick();}
  
  robotServer.log("2. Turning Right...");
  motors::setTargetRotation(-90);
  while (motors::isInAction) {motors::tick();}

  robotServer.log("3. Going straight...");
  motors::setTargetPosition(18);
  while (motors::isInAction) {motors::tick();}

  robotServer.log("4. Turning left...");
  motors::setTargetRotation(90);
  while (motors::isInAction) {motors::tick();}
}
