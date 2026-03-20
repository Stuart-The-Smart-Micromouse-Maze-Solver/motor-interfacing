#include <Arduino.h>
#include "motors.h"
#include "encoders.h"
#include "gyro_heading.h"
#include "motion.h"
#include "config.h"
#include "server.h"
#include "distance.h"
#include <Wire.h>
#include <FastLED.h>

CRGB leds[1];

void SetLED(CRGB col) {
  leds[0] = col;
  FastLED.show();
}
RobotServer robotServer(
  &motors::rotationPID,
  &motors::positionPID,
  &motors::rightVelocityPID,
  &motors::leftVelocityPID
);
volatile bool needsRestart = false;


void startButtonClicked() {
  motors::rightVelocityPID.setEnabled(true);
  motors::leftVelocityPID.setEnabled(true);
}
void stopButtonClicked() {
  motors::rightVelocityPID.setEnabled(false);
  motors::leftVelocityPID.setEnabled(false);
  motors::stop();
}
void restartButtonClicked() {
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
  // WS2812  or  NEOPIXEL?
  FastLED.addLeds<WS2812 , RGB_LED_PIN, GRB>(leds, 1).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(60);
  SetLED(CRGB::Black);


  Serial.begin(115200);
  delay(500);

  SetLED(CRGB::Blue);  // show blue during initialization

  motors::init();
  encodersInit();
  // //motionInit();

  Serial.println("Init complete");

  if (distanceInit()) {
    Serial.println("Distance Sensors: OK");
  } else {
    Serial.println("Distance Sensors: FAILED (Check wiring/XSHUT)");
    SetLED(CRGB::Red);
  }

  if (gyroInit()) {
    gyroQuickBiasCal(2000);  // ~2s bias calibration at boot
    Serial.println("Gyro OK");
  } else {
    Serial.println("Gyro NOT found (heading hold disabled)");
    SetLED(CRGB::Red);
  }


  /*
  xTaskCreatePinnedToCore(
    [](void* p){ 
        robotServer.begin(
          WIFI_SSID, 
          WIFI_PWD, 
          startButtonClicked,
          stopButtonClicked,
          restartButtonClicked,
          setTargetPosition,
          setTargetTurn
        );
        // for(;;) { vTaskDelay(1000 / portTICK_PERIOD_MS); distanceUpdateAll(); } // Keep task alive  // UPDATE TOF IN SEPARATE CORE CAUSE ITS SO SLOW
        TickType_t lastWakeTime = xTaskGetTickCount();
        uint32_t distanceCounter = 0;
        for (;;) {
          gyroCache(); // fast I2C read (~50-100us), offloaded from Core 1
          if (++distanceCounter >= 10) {
            distanceCounter = 0;
            distanceUpdateAll(); // slower ToF read, run at 100ms
          }
          vTaskDelayUntil(&lastWakeTime, 10 / portTICK_PERIOD_MS); // 10ms = 100Hz
        };
    },
    "WebServerTask",
    8192,  // Stack size
    NULL,
    1,     // Priority
    NULL,
    0      // Core 0
  );
  */

  // UPDATED TO INCLUDE FREQUENCY LOGGING. TEMP???
  xTaskCreatePinnedToCore(
    [](void* p){ 
        robotServer.begin(
          WIFI_SSID, 
          WIFI_PWD, 
          startButtonClicked,
          stopButtonClicked,
          restartButtonClicked,
          setTargetPosition,
          setTargetTurn
        );

        TickType_t lastWakeTime = xTaskGetTickCount();
        uint32_t distanceCounter = 0;

        // --- Frequency Tracking Variables ---
        uint32_t gyroExecCount = 0;
        uint32_t distExecCount = 0;
        uint32_t lastFreqLogMs = millis();
        // ------------------------------------

        for (;;) {
          // 1. Run Gyro Cache
          gyroCache(); 
          gyroExecCount++; // Increment gyro counter

          // 2. Run Distance Update (every 10th loop)
          // if (++distanceCounter >= 10) {
          if (++distanceCounter >= 5) { // every 5th loop
            distanceCounter = 0;
            distanceUpdateAll(); 
            distExecCount++; // Increment distance counter
          }

          // 3. Log Frequency every 1 second (1000ms)
          uint32_t now = millis();
          if (now - lastFreqLogMs >= 1000) {
            float elapsedSec = (now - lastFreqLogMs) / 1000.0f;
            
            // Calculate Hz (Count / Seconds)
            float gyroHz = gyroExecCount / elapsedSec;
            float distHz = distExecCount / elapsedSec;

            // Send to log
            String logMsg = "Freq - Gyro: " + String(gyroHz, 1) + "Hz, Dist: " + String(distHz, 1) + "Hz";
            robotServer.log(logMsg);

            // Reset for next window
            gyroExecCount = 0;
            distExecCount = 0;
            lastFreqLogMs = now;
          }

          // vTaskDelayUntil(&lastWakeTime, 10 / portTICK_PERIOD_MS); // 100hz
          vTaskDelayUntil(&lastWakeTime, 2 / portTICK_PERIOD_MS); // 500hz
        };
    },
    "WebServerTask",
    8192,  // Stack size
    NULL,
    1,     // Priority
    NULL,
    0      // Core 0
  );


  leds[0] = CRGB::Black;  // turn off after initializing
  FastLED.show(); 
}

void loop()
{
  uint32_t nowMs = millis();

  uint32_t temp_timer_outer = millis();


  while (!needsRestart) {
    nowMs = millis();

    motors::tick(); // handles frequency correctly

    // robotServer.log("Gyro IMU: " + String(gyroHeadingDeg()));
    // robotServer.log("Gyro ENC: " + String(COUNTS_OFFSET_PER_DEG * (readEncoderCounts(rightEncoder) - readEncoderCounts(leftEncoder))));
    if (nowMs > temp_timer_outer + 100) {
      temp_timer_outer = nowMs;
      // 10hz temp loop

      // distanceUpdateAll();


      // robotServer.log("Gyro IMU/ENC: " + String(readDeg()) + " / " + String(COUNTS_OFFSET_PER_DEG * (readEncoderCounts(rightEncoder) - readEncoderCounts(leftEncoder))));
      // robotServer.log("correctionAngle: " + String(motors::tof_correction_angle));
    }
  }
  needsRestart = false;

  
  robotServer.log("1. Going straight...");
  motors::performingTurn = false;
  motors::setTargetPosition(18);
  while (motors::isInAction) {motors::tick();}
  
  robotServer.log("2. Turning Right...");
  motors::performingTurn = true;
  motors::setTargetRotation(90);
  while (motors::isInAction) {motors::tick();}

  robotServer.log("3. Going straight...");
  motors::performingTurn = false;
  motors::setTargetPosition(18);
  while (motors::isInAction) {motors::tick();}

  robotServer.log("4. Turning Right...");
  motors::performingTurn = true;
  motors::setTargetRotation(90);
  while (motors::isInAction) {motors::tick();}

  robotServer.log("5. Going straight...");
  motors::performingTurn = false;
  motors::setTargetPosition(18);
  while (motors::isInAction) {motors::tick();}

  robotServer.log("4. Turning Left...");
  motors::performingTurn = true;
  motors::setTargetRotation(-90);
  while (motors::isInAction) {motors::tick();}
}
