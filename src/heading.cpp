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
static uint32_t lastUpdate = 0;

bool gyroInit()
{
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!gyro.begin()) {
    gyroValid = false;
    return false;
  }

  gyro.setRange(GYRO_RANGE_250DPS);

  gyroValid = true;
  integratedDeg = 0.0f;
  zeroOffsetDeg = 0.0f;
  gyroBiasZ = 0.0f;
  lastUpdate = micros();

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

void gyroUpdate()
{
  if (!gyroValid) return;

  uint32_t now = micros();
  float dt = (now - lastUpdate) * 1e-6f;
  lastUpdate = now;

  sensors_event_t g;
  if (!gyro.getEvent(&g)) return;

  float gz = g.gyro.z - gyroBiasZ; // rad/s

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