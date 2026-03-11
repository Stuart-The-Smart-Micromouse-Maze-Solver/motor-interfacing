#include <Arduino.h>
#include <Wire.h>
#include <FastLED.h>
#include "esp_system.h"

#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "gyro_heading.h"
#include "distance.h"
#include "server.h"
#include "maze_nav.h"

CRGB leds[1];

static void SetLED(const CRGB& col) {
  leds[0] = col;
  FastLED.show();
}

RobotServer robotServer(
  &motors::rotationPID,
  &motors::positionPID,
  &motors::rightVelocityPID,
  &motors::leftVelocityPID
);

void startButtonClicked() {
  maze_nav::observeCurrentCell();
}

void stopButtonClicked() {
  maze_nav::clearQueue();
  motors::stop();
}

void restartButtonClicked() {
  maze_nav::clearQueue();
  motors::stop();
  maze_nav::resetToStart();
  maze_nav::observeCurrentCell();
}

void setTargetPosition(int cellCount) {
  robotServer.log("WEB /pos cells=" + String(cellCount));
  motors::setTargetPosition(cellCount * 18.0f);
}

void setTargetTurn(float deg) {
  robotServer.log("WEB /turn deg=" + String(deg));
  motors::setTargetRotation(deg);
}

static void serverTask(void* /*pvParameters*/) {
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
  for (;;) {
    distanceUpdateAll();
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(60));
  }
}

void setup() {
  FastLED.addLeds<WS2812, RGB_LED_PIN, GRB>(leds, 1).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(60);
  SetLED(CRGB::Black);

  Serial.begin(115200);
  delay(500);

  esp_reset_reason_t reason = esp_reset_reason();
  Serial.print("Reset reason: ");
  Serial.println((int)reason);

  SetLED(CRGB::Blue);

  motors::init();
  encodersInit();
  Serial.println("Init complete");

  if (distanceInit()) {
    Serial.println("Distance Sensors: OK");
  } else {
    Serial.println("Distance Sensors: FAILED (Check wiring/XSHUT)");
    SetLED(CRGB::Red);
  }

  if (gyroInit()) {
    gyroQuickBiasCal(2000);
    Serial.println("Gyro OK");
  } else {
    Serial.println("Gyro NOT found (heading hold disabled)");
    SetLED(CRGB::Red);
  }

  maze_nav::init();
  delay(100);
  distanceUpdateAll();
  maze_nav::observeCurrentCell();

  xTaskCreatePinnedToCore(
    serverTask,
    "WebServerTask",
    12288,
    nullptr,
    1,
    nullptr,
    0
  );

  SetLED(CRGB::Black);
}

void loop() {
  motors::tick();
  maze_nav::tick();
  delay(1);
}
