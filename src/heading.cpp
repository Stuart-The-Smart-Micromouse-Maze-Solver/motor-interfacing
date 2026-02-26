#include <Wire.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_Sensor.h>
#include "config.h"
#include "gyro_heading.h"

static Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

static bool gyroValid = false;
static float headingDeg = 0.0f;
static float gyroBiasZ = 0.0f;
static uint32_t lastIMUUpdate = 0;

bool gyroInit()
{
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!gyro.begin()) {
    gyroValid = false;
    return false;
  }

  gyroValid = true;
  gyro.setRange(GYRO_RANGE_250DPS);

  headingDeg = 0.0f;
  gyroBiasZ = 0.0f;
  lastIMUUpdate = micros();
  return true;
}

bool gyroIsValid() { return gyroValid; }

float gyroHeadingDeg() { return headingDeg; }

void gyroResetHeading(float h)
{
  headingDeg = h;
  lastIMUUpdate = micros();
}

void gyroQuickBiasCal(uint16_t samples)
{
  if (!gyroValid) return;

  float sumZ = 0.0f;
  for (uint16_t i = 0; i < samples; i++) {
    sensors_event_t g;
    if (gyro.getEvent(&g)) sumZ += g.gyro.z;
    delay(2);
  }
  gyroBiasZ = sumZ / samples;
  gyroResetHeading(0.0f);
}

void gyroUpdate()
{
  if (!gyroValid) return;

  uint32_t now = micros();
  float dt = (now - lastIMUUpdate) / 1000000.0f;
  lastIMUUpdate = now;

  sensors_event_t g;
  if (!gyro.getEvent(&g)) return;

  float gz = g.gyro.z - gyroBiasZ; // rad/s
  headingDeg += (gz * 180.0f / PI) * dt;

  // while (headingDeg >= 360.0f) headingDeg -= 360.0f;
  // while (headingDeg < 0.0f)    headingDeg += 360.0f;
  while (headingDeg >= 180.0f) headingDeg -= 360.0f;
  while (headingDeg < -180.0f)    headingDeg += 360.0f;
}

float angleDiffDeg(float target, float current)
{
  float d = target - current;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}
