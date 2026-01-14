#include "config.h"
#include "motion.h"
#include "motors.h"
#include "encoders.h"
#include "gyro_heading.h"
#include "control.h"

// Your speed PID constants
static PID leftPID  = {1.0f, 0.2f, 0.00f, 0, 0, 40.0f, 0};
static PID rightPID = {1.0f, 0.2f, 0.00f, 0, 0, 40.0f, 0};

void moveForwardCmClean(float distanceCm, float speedRPM)
{
  const uint32_t TIMEOUT_MS = 8000;
  const uint32_t LOOP_MS = 10;

  const float RAMP_UP_CM = 4.0f;
  const float RAMP_DOWN_CM = 10.0f;
  const float MIN_RPM = 60.0f;
  const float STOP_TOL_CM = 0.15f;

  // smoother steering
  const float HEADING_KP = 1.2f; //lower = reduces how hard it reacts
  const float HEADING_DEADBAND_DEG = 1.0f; // smaller = more sensitive (0.4-1.0)
  const float HEADING_CORR_CLAMP = 30.0f; // steering authority -> too high = sharp corrections (25-60)
  const float CORR_ALPHA = 0.05f; // big = faster response, but more jitter

  const float DRIFT_GAIN_RPM_PER_CM = 18.0f;

  int64_t startLeft  = readEncoderCounts(leftEncoder);
  int64_t startRight = readEncoderCounts(rightEncoder);

  resetPID(&leftPID);
  resetPID(&rightPID);

  leftEncoder.prevTime  = micros();
  rightEncoder.prevTime = micros();
  leftEncoder.prevCounts  = startLeft;
  rightEncoder.prevCounts = startRight;

  uint32_t startTime  = millis();
  uint32_t lastUpdate = millis();

  // stabilize heading
  for (int i = 0; i < 5; i++) { gyroUpdate(); delay(5); }
  float targetHeading = gyroHeadingDeg();

  float corrRPM_f = 0.0f;

  // kick
  setMotorCommand(&leftMotor, 140);
  setMotorCommand(&rightMotor, 140);
  delay(70);

  while (true)
  {
    if (millis() - startTime > TIMEOUT_MS) break;

    if (millis() - lastUpdate < LOOP_MS) { delay(1); continue; }
    float dt = (millis() - lastUpdate) / 1000.0f;
    lastUpdate = millis();

    gyroUpdate();

    int64_t lc = readEncoderCounts(leftEncoder);
    int64_t rc = readEncoderCounts(rightEncoder);

    float leftDistCm  = (lc - startLeft)  * metersPerCountCal() * 100.0f;
    float rightDistCm = (rc - startRight) * metersPerCountCal() * 100.0f;

    float avgDistCm = 0.5f * (leftDistCm + rightDistCm);
    float remaining = distanceCm - avgDistCm;

    if (remaining <= STOP_TOL_CM) break;

    float baseRPM = speedRPM;

    // baseRPM -> how fast we want to go before corrections

    if (avgDistCm < RAMP_UP_CM) {
      float s = avgDistCm / max(0.001f, RAMP_UP_CM);
      baseRPM = MIN_RPM + s * (speedRPM - MIN_RPM);
    }
    if (remaining < RAMP_DOWN_CM) {
      float s = remaining / max(0.001f, RAMP_DOWN_CM);
      float downRPM = MIN_RPM + s * (speedRPM - MIN_RPM);
      baseRPM = min(baseRPM, downRPM);
    }
    baseRPM = constrain(baseRPM, MIN_RPM, speedRPM);

    float corrRaw = 0.0f;

    if (gyroIsValid()) {
      float hErr = angleDiffDeg(targetHeading, gyroHeadingDeg());
      if (fabs(hErr) < HEADING_DEADBAND_DEG) hErr = 0.0f;

      corrRaw = HEADING_KP * hErr;
      corrRaw = constrain(corrRaw, -HEADING_CORR_CLAMP, HEADING_CORR_CLAMP);
    } else {
      float driftCm = leftDistCm - rightDistCm;
      corrRaw = constrain(driftCm * DRIFT_GAIN_RPM_PER_CM,
                          -HEADING_CORR_CLAMP, HEADING_CORR_CLAMP);
    }

    corrRPM_f += CORR_ALPHA * (corrRaw - corrRPM_f);

    float leftTargetRPM  = baseRPM - corrRPM_f;
    float rightTargetRPM = baseRPM + corrRPM_f;

    leftTargetRPM  = constrain(leftTargetRPM,  0.0f, speedRPM * 1.4f);
    rightTargetRPM = constrain(rightTargetRPM, 0.0f, speedRPM * 1.4f);

    updateMotorSpeeds(dt);
    setMotorSpeedRPM(&leftMotor, &leftEncoder, &leftPID, leftTargetRPM);
    setMotorSpeedRPM(&rightMotor, &rightEncoder, &rightPID, rightTargetRPM);
  }

  brakeStop(140);
  delay(120);
}

void autoDemoLoop()
{
  static const float seq_cm[] = {18.0f, 36.0f, 54.0f, 72.0f};
  static const size_t N = sizeof(seq_cm) / sizeof(seq_cm[0]);
  static size_t idx = 0;

  static bool inRest = false;
  static uint32_t restStart = 0;

  const float SPEED_RPM = 200.0f;
  const uint32_t REST_MS = 10000UL;

  if (!inRest) {
    moveForwardCmClean(seq_cm[idx], SPEED_RPM);
    stopMotors();
    inRest = true;
    restStart = millis();
    idx = (idx + 1) % N;
  } else {
    if (millis() - restStart >= REST_MS) inRest = false;
  }
}

void turnDegreesGyro(float degrees, float speedRPM)
{}