#include <Arduino.h>
#include "motors.h"
#include "encoders.h"
#include "gyro_heading.h"
#include "motion.h"
#include "config.h"
#include "StuartServer.h"
#include "distance.h"
#include "autotune.h"
#include <Wire.h>
#include <FastLED.h>

CRGB leds[1];

void SetLED(CRGB col) {
  leds[0] = col;
  FastLED.show();
}

StuartServer server;

volatile bool needsRestart   = false;
volatile bool needsCalibrate = false;


// ═══════════════════════════════════════════════════════════════════
//  Server callbacks
// ═══════════════════════════════════════════════════════════════════

void onStart() {
  motors::rightVelocityPID.setEnabled(true);
  motors::leftVelocityPID.setEnabled(true);
}

void onStop() {
  motionAbort();
  motors::rightVelocityPID.setEnabled(false);
  motors::leftVelocityPID.setEnabled(false);
  motors::stop();
}

void onRestart() {
  needsRestart = true;
}

void onZero() {
  motionAbort();
  motors::zero();
}

void onPosition(int cellCount) {
  motors::setTargetPosition(cellCount * CELL_SIZE_CM);
}

void onRotation(int deg) {
  motors::setTargetRotation(static_cast<float>(deg));
}

// Populate from your PIDController getters.
// PidSnapshot fields: actual=getFeedback(), target=getTarget(),
//                     error=getError(), kp=getP(), ki=getI(), kd=getD()
template<typename PID>
static PidSnapshot pidSnap(PID* p) {
  PidSnapshot s;
  s.actual = p->getFeedback();
  s.target = p->getTarget();
  s.error  = p->getError();
  s.kp     = p->getP();
  s.ki     = p->getI();
  s.kd     = p->getD();
  return s;
}

TelemetrySnapshot onData() {
  TelemetrySnapshot s;

  // ── Heading & position ──────────────────────────────────────────
  // TODO: replace with your actual gyro + encoder accessors
  // s.heading  = gyroGetHeading();
  // s.position = motors::positionPID.getFeedback();

  // ── Encoders ────────────────────────────────────────────────────
  // TODO: replace with your encoder count accessors
  // s.encoder_left  = encoderLeftCount();
  // s.encoder_right = encoderRightCount();

  // ── ToF ─────────────────────────────────────────────────────────
  // TODO: fill in from distance.h
  // s.tof_l = getDistanceLeft();
  // s.tof_f = getDistanceFront();
  // s.tof_r = getDistanceRight();

  // ── PID snapshots ───────────────────────────────────────────────
  s.pos_pid   = pidSnap(&motors::positionPID);
  s.rot_pid   = pidSnap(&motors::rotationPID);
  s.vel_r_pid = pidSnap(&motors::rightVelocityPID);
  s.vel_l_pid = pidSnap(&motors::leftVelocityPID);

  return s;
}

void onPid(const String& name, float p, float i, float d) {
  // TODO: replace with your PID setter calls, e.g.:
  // if      (name == "position") { motors::positionPID.setGains(p, i, d); }
  // else if (name == "rotation") { motors::rotationPID.setGains(p, i, d); }
  // else if (name == "rvel")     { motors::rightVelocityPID.setGains(p, i, d); }
  // else if (name == "lvel")     { motors::leftVelocityPID.setGains(p, i, d); }
  server.log("PID " + name + ": P=" + String(p,4) + " I=" + String(i,4) + " D=" + String(d,4));
}

void onInstructions(const String& seq) {
  motionExecute(seq);
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

  // ── Assign server handlers (before the task calls begin()) ──────
  server.init(80);
  server.onStart       (onStart);
  server.onStop        (onStop);
  server.onRestart     (onRestart);
  server.onZero        (onZero);
  server.onPosition    (onPosition);
  server.onRotation    (onRotation);
  server.onData        (onData);
  server.onPid         (onPid);
  server.onInstructions(onInstructions);
  // server.onMaze([]() -> MazeGraph { return yourMazeGraph; }); // TODO

  // ── Core 0: WiFi + server start + Gyro + ToF task ───────────────
  xTaskCreatePinnedToCore(
    [](void* p){
        // WiFi
        WiFi.begin(WIFI_SSID, WIFI_PWD);
        Serial.print("Connecting to WiFi");
        while (WiFi.status() != WL_CONNECTED) {
          delay(200);
          Serial.print('.');
        }
        Serial.println();
        Serial.print("IP: "); Serial.println(WiFi.localIP());

        server.begin();
        server.log("Stuart online at " + WiFi.localIP().toString());

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
          if (now - lastFreqLogMs >= 5000) {
            float elapsedSec = (now - lastFreqLogMs) / 1000.0f;
            float gyroHz = gyroExecCount / elapsedSec;
            float distHz = distExecCount / elapsedSec;
            server.log("Freq — Gyro: " + String(gyroHz, 1) + "Hz, Dist: " + String(distHz, 1) + "Hz");
            gyroExecCount = 0;
            distExecCount = 0;
            lastFreqLogMs = now;
          }

          vTaskDelayUntil(&lastWakeTime, 2 / portTICK_PERIOD_MS);
        }
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
//  Main loop (Core 1) — PID cascade + motion executor
// ═══════════════════════════════════════════════════════════════════
void loop()
{
  while (!needsRestart && !needsCalibrate) {
    motors::tick();
    motionUpdate();
    yield();
  }

  if (needsCalibrate) {
    needsCalibrate = false;
    motionAbort();

    SetLED(CRGB::Yellow);
    CalibrationResult cal = runCalibration(
      [](const String& msg) { server.log(msg); }
    );

    if (cal.success) {
      SetLED(CRGB::Green);
      server.log("Calibration succeeded! Gains applied.");
    } else {
      SetLED(CRGB::Red);
      server.log("Calibration FAILED.");
    }
    delay(1000);
    SetLED(CRGB::Black);
  }

  if (needsRestart) {
    needsRestart = false;

    motionAbort();
    motors::zero();

    server.log("Executing demo: F,R,F,R,F,R,F (square)");
    motionExecute("F,R,F,R,F,R,F");

    while (motionIsBusy()) {
      motors::tick();
      motionUpdate();
      yield();
    }
    server.log("Demo complete.");
  }
}