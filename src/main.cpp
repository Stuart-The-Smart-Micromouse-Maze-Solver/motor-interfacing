#include <Arduino.h>
#include "motors.h"
#include "encoders.h"
#include "gyro_heading.h"
#include "motion.h"
#include "config.h"
#include "server.h"
#include "distance.h"
#include "autotune.h"
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
volatile bool needsCalibrate = false;

// ═══════════════════════════════════════════════════════════════════
//  Server callbacks
// ═══════════════════════════════════════════════════════════════════
void startButtonClicked() {
  motors::rightVelocityPID.setEnabled(true);
  motors::leftVelocityPID.setEnabled(true);
}

void stopButtonClicked() {
  motionAbort();
  motors::rightVelocityPID.setEnabled(false);
  motors::leftVelocityPID.setEnabled(false);
  motors::stop();
}

void restartButtonClicked() {
  needsRestart = true;
}

void setTargetPosition(int cellCount) {
  motors::setTargetPosition(cellCount * CELL_SIZE_CM);
}

void setTargetTurn(float deg) {
  motors::setTargetRotation(deg);
}

void zeroButtonClicked() {
  motionAbort();
  motors::zero();
}


// ═══════════════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════════════
void setup()
{
  FastLED.addLeds<WS2812, RGB_LED_PIN, GRB>(leds, 1).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(60);
  SetLED(CRGB::Black);

  Serial.begin(115200);
  delay(500);

  SetLED(CRGB::Blue);

  motors::init();
  encodersInit();
  motionInit();

  Serial.println("Init complete");

  if (distanceInit()) {
    Serial.println("Distance Sensors: OK");
  } else {
    Serial.println("Distance Sensors: FAILED");
    SetLED(CRGB::Red);
  }

  if (gyroInit()) {
    gyroQuickBiasCal(2000);
    Serial.println("Gyro OK");
  } else {
    Serial.println("Gyro NOT found");
    SetLED(CRGB::Red);
  }

  // ── Core 0: WiFi + Gyro + ToF task ────────────────────────────
  xTaskCreatePinnedToCore(
    [](void* p){ 
        robotServer.begin(
          WIFI_SSID,
          WIFI_PWD,
          startButtonClicked,
          stopButtonClicked,
          restartButtonClicked,
          setTargetPosition,
          setTargetTurn,
          [](const String& seq) -> bool { return motionExecute(seq); },
          []() { needsCalibrate = true; },
          zeroButtonClicked
        );

        TickType_t lastWakeTime = xTaskGetTickCount();
        uint32_t distanceCounter = 0;

        uint32_t gyroExecCount = 0;
        uint32_t distExecCount = 0;
        uint32_t lastFreqLogMs = millis();

        for (;;) {
          gyroCache(); 
          gyroExecCount++;

          if (++distanceCounter >= 5) {
            distanceCounter = 0;
            distanceUpdateAll(); 
            distExecCount++;
          }

          uint32_t now = millis();
          if (now - lastFreqLogMs >= 5000) {  // Log every 5s (less spam)
            float elapsedSec = (now - lastFreqLogMs) / 1000.0f;
            float gyroHz = gyroExecCount / elapsedSec;
            float distHz = distExecCount / elapsedSec;
            String logMsg = "Freq - Gyro: " + String(gyroHz, 1) + "Hz, Dist: " + String(distHz, 1) + "Hz";
            // robotServer.log(logMsg);
            gyroExecCount = 0;
            distExecCount = 0;
            lastFreqLogMs = now;
          }

          vTaskDelayUntil(&lastWakeTime, 2 / portTICK_PERIOD_MS);
        };
    },
    "SensorTask",
    8192,
    NULL,
    1,
    NULL,
    0  // Core 0
  );

  SetLED(CRGB::Black);
}


// ═══════════════════════════════════════════════════════════════════
//  Main loop (Core 1) — runs PID cascade + motion executor
// ═══════════════════════════════════════════════════════════════════
void loop()
{
  uint32_t logTimer = millis();

  while (!needsRestart && !needsCalibrate) {
    // Run the PID cascade at maximum rate
    motors::tick();

    // Advance the motion instruction executor
    motionUpdate();

    // Periodic telemetry
    if (millis() - logTimer > 200) {
      logTimer = millis();
    }

    yield();
  }

  if (needsCalibrate) {
    // ── Run auto-calibration ─────────────────────────────────────
    needsCalibrate = false;
    motionAbort();

    SetLED(CRGB::Yellow);
    CalibrationResult cal = runCalibration(
      [](const String& msg) { robotServer.log(msg); }
    );

    if (cal.success) {
      SetLED(CRGB::Green);
      robotServer.log("Calibration succeeded! Gains applied.");
    } else {
      SetLED(CRGB::Red);
      robotServer.log("Calibration FAILED.");
    }
    delay(1000);
    SetLED(CRGB::Black);
  }

  if (needsRestart) {
    // ── Run demo sequence ────────────────────────────────────────
    needsRestart = false;

    // Clear any in-flight motion and zero pose before starting sequence.
    // Without this, if isInAction==true from a prior move, setTargetPosition()
    // silently returns and the first primitive is skipped entirely.
    motionAbort();
    motors::zero();

    robotServer.log("Executing demo: F,R,F,R,F,R,F (square)");
    motionExecute("F,R,F,R,F,R,F");

    while (motionIsBusy()) {
      motors::tick();
      motionUpdate();
      yield();
    }
    robotServer.log("Demo complete.");
  }
}
