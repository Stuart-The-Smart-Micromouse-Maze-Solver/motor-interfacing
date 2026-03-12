#include <Wire.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_Sensor.h>
#include "config.h"
#include "gyro_heading.h"

static Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

static bool gyroValid = false;

static float integratedDeg = 0.0f;   // continuous integration
static float zeroOffsetDeg = 0.0f;   // reference heading

static float gyroBiasZ = 0.0f;
static uint32_t lastIMUUpdate = 0;

// Cached bias-corrected gz (rad/s), written by Core 0 via gyroCache()
static volatile float cachedGz = 0.0f;
static volatile bool  cacheReady = false;
extern TwoWire I2CBus1;

bool gyroInit()
{
  
  if (!gyro.begin(33U, &I2CBus1)) {
    gyroValid = false;
    return false;
  }

  gyro.setRange(GYRO_RANGE_250DPS);

  gyroValid = true;
  integratedDeg = 0.0f;
  zeroOffsetDeg = 0.0f;
  gyroBiasZ = 0.0f;
  lastIMUUpdate = micros();

  return true;
}

void gyroQuickBiasCal(uint16_t samples)
{
  if (!gyroValid) return;

  float sum = 0.0f;

  for (uint16_t i = 0; i < samples; i++) {
    sensors_event_t g;
    if (gyro.getEvent(&g)) sum += g.gyro.z;
    delay(2);
  }

  gyroBiasZ = sum / samples;

  integratedDeg = 0.0f;
  zeroOffsetDeg = 0.0f;
}

// Called from Core 0 task — performs the blocking I2C read and caches result.
void gyroCache()
{
  if (!gyroValid) return;

  sensors_event_t g;
  if (!gyro.getEvent(&g)) return;

  cachedGz = g.gyro.z - gyroBiasZ; // rad/s, bias-corrected
  if (!cacheReady) {
    lastIMUUpdate = micros();  // anchor dt clock here, not at gyroInit()
    cacheReady = true;
  }
}

// Lightweight integration using cached gz — no I2C, safe to call from Core 1.
void gyroUpdate()
{
  if (!gyroValid || !cacheReady) return;

  uint32_t now = micros();
  float dt = (now - lastIMUUpdate) * 1e-6f;
  lastIMUUpdate = now;

  float gz = cachedGz; // read cached value
  if (fabsf(gz) < 0.01f) return;  // below noise floor — skip integration
  integratedDeg += gz * 57.2957795f * dt; // rad → deg
}


float readDeg()
{
  float relative = integratedDeg - zeroOffsetDeg;

  // while (relative > 180.0f)  relative -= 360.0f;
  // while (relative < -180.0f) relative += 360.0f;

  return relative;
}

void resetDeg()
{
  zeroOffsetDeg = integratedDeg;
}